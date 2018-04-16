package com.example.guardianangelroverv3;

import android.Manifest;
import android.app.Dialog;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.indooratlas.android.sdk.resources.IAFloorPlan;
import com.indooratlas.android.sdk.resources.IALatLng;
import com.indooratlas.android.wayfinding.IARoutingLeg;
import com.indooratlas.android.wayfinding.IAWayfinder;

import java.io.InputStream;
import java.util.ArrayList;

public class MainActivity extends AppCompatActivity implements OnMapReadyCallback {

    private final String TAG = "MainActivity";
    private final int PERMISSION_CODE = 1;

    // singleton objects which contact server and determine location
    private static ServerLink myserver;
    private static IndoorAtlas atlas;

    // name of rover and active user
    private String name = "ROVER";

    private TextView textView;

    // mapping
    private GoogleMap mMap;
    private Marker mMarker;
    private Marker mMarkerOther;
    private MapView mMapView;
    private GroundOverlay mGroundOverlay = null;

    // locations and wayfinding
    private IAWayfinder mWayfinder;
    private LatLng mLocation;
    private LatLng mDestination;
    private int mLocationFloor;
    private int mDestinationFloor;
    private Polyline mPath;
    private Polyline mPathCurrent;
    private IARoutingLeg[] mCurrentRoute;

    // TODO: set home_lat and home_lon
    private double home_lat = 1;
    private double home_lon = 1;

    private static final float HUE_IABLUE = 200.0f;

    private Bundle mSavedInstanceState;

    final Handler handler = new Handler();
    private Runnable runnableCode;

    public final int STATE_REST_HOME = 0;
    public final int STATE_REQUEST = 1;
    public final int STATE_ARRIVED = 2;
    public final int STATE_ENDED = 3;
    public final int STATE_EMERGENCY = 4;

    private int current_state = STATE_REST_HOME;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSavedInstanceState = savedInstanceState;

        final Dialog dialog = new Dialog(MainActivity.this);
        dialog.setContentView(R.layout.dialog_ip);
        dialog.setTitle("Set Server IP Address");

        Button button = dialog.findViewById(R.id.okbutton);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                EditText edit = dialog.findViewById(R.id.ipedittext);
                String text = edit.getText().toString();
                dialog.dismiss();
                myserver.setIPAddress(text);
            }
        });

        dialog.show();

        // set CurrentActivity to MainActivity
        CurrentActivity.getInstance().setCurrentActivity(1, this);

        // prevent the screen going to sleep while app is on foreground
        findViewById(android.R.id.content).setKeepScreenOn(true);

        textView = findViewById(R.id.textView2);

        // Check location permissions
        String[] neededPermissions = {
                Manifest.permission.CHANGE_WIFI_STATE,
                Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
        };
        boolean granted = true;
        for (String perm : neededPermissions) {
            granted = granted && ContextCompat.checkSelfPermission(this, perm) ==
                    PackageManager.PERMISSION_GRANTED;
        }
        if (!granted) {
            // permission not granted for one or more of the permissions
            // request permission
            ActivityCompat.requestPermissions(this, neededPermissions, PERMISSION_CODE);
        } else {
            // permission has been granted
            initialize();
        }
    }

    public void initialize() {
        // set up server link
        myserver = ServerLink.getInstance();

        // Initialize IndoorAtlas
        atlas = IndoorAtlas.getInstance();

        // start setting up map
        mMapView = findViewById(R.id.mapView);
        mMapView.onCreate(mSavedInstanceState);
        mMapView.getMapAsync(this);

        // load JSON wayfinding graph
        String graphJSON = loadGraphJSON();
        if (graphJSON == null) {
            Toast.makeText(this, "Could not find wayfinding_graph.json from raw " +
                    "resources folder. Cannot do wayfinding.", Toast.LENGTH_LONG).show();
        } else {
            mWayfinder = IAWayfinder.create(this, graphJSON);
        }

        // check state from server every second
        runnableCode = new Runnable() {
            @Override
            public void run() {
                Log.d("Handlers", "Called on main thread");
                if (myserver.other_lat == 0 && myserver.other_lon == 0) {
                    current_state = STATE_REST_HOME;
                    myserver.getRequestUserCoords();
                } else {
                    myserver.getRequestState();
                    if (myserver.requested) {
                        Log.d(TAG, "requested");
                        current_state = STATE_REQUEST;
                        updateMap();
                    }
                    if (myserver.emergency) {
                        Log.d(TAG, "emergency");
                        current_state = STATE_EMERGENCY;
                        findViewById(R.id.button2).setBackgroundColor(Color.RED);
                    }
                    if (myserver.arrived) {
                        Log.d(TAG, "arrived");
                        current_state = STATE_ARRIVED;
                        // don't need to do anything?
                    }
                    if (myserver.ended) {
                        Log.d(TAG, "ended");
                        current_state = STATE_ENDED;
                        // send rover back home using location specified by user
                        atlas.lat = myserver.rover_lat;
                        atlas.lon = myserver.rover_lon;
                        myserver.other_lat = home_lat;
                        myserver.other_lon = home_lon;
                        myserver.user_name = "HOME";
                        updateMap();
                    }
                }
                handler.postDelayed(this, 5000);
            }
        };
        handler.post(runnableCode);
    }

    // handle response from permissions request
    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode) {
            case PERMISSION_CODE: {
                boolean granted = true;
                for (int result : grantResults) {
                    granted = granted && result == PackageManager.PERMISSION_GRANTED;
                }
                if (granted) {
                    initialize();
                } else {
                    // permission was denied - cannot continue with app
                    Log.d(TAG, "Permission was denied, cannot continue with app.");
                }
            }
        }
    }

    public void updateServer() {
        // create JSON string with location data
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_ROVER));
        jsonlist.add(new StringPair(ServerLink.NAME, name));
        jsonlist.add(new StringPair(ServerLink.LAT, Double.toString(atlas.getLat())));
        jsonlist.add(new StringPair(ServerLink.LON, Double.toString(atlas.getLon())));
        myserver.request(jsonlist);

        // check server for user's location
        myserver.getRequestUserCoords();

        // check server if emergency is active
        myserver.getRequestState();
    }

    // set up map
    @Override
    public void onMapReady(GoogleMap googleMap) {
        Log.d("Map", "Map ready");
        mMap = googleMap;
        mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                atlas.lat = latLng.latitude;
                atlas.lon = latLng.longitude;
                updateMap();
            }
        });
        //updateServer();
        updateMap();
    }

    // update the locations and route on the map, also check if an emergency is active
    public void updateMap() {

        if (mMap == null) {
            // location received before map is initialized, ignoring update here
            return;
        }

        Log.d("Map", "Updating map");

        mLocation = new LatLng(atlas.getLat(), atlas.getLon());
        mLocationFloor = 2; //atlas.getFloor();
        if (mMarker == null) {
            if (mMap != null) {
                mMarker = mMap.addMarker(new MarkerOptions().position(mLocation)
                        .icon(BitmapDescriptorFactory.defaultMarker(HUE_IABLUE))
                        .title("ROVER"));
                mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(mLocation, 17.0f));
            }
        } else {
            mMarker.setPosition(mLocation);
        }

        if (myserver.other_lat != 0 && myserver.other_lon != 0) {
            mDestination = new LatLng(myserver.other_lat, myserver.other_lon);
            mDestinationFloor = mLocationFloor;
            if (mMarkerOther == null) {
                if (mMap != null) {
                    mMarkerOther = mMap.addMarker(new MarkerOptions().position(mDestination)
                            .icon(BitmapDescriptorFactory.defaultMarker(100))
                            .title(myserver.user_name));
                    mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(mDestination, 17.0f));
                }
            } else {
                mMarkerOther.setTitle(myserver.user_name);
                mMarkerOther.setPosition(mDestination);
            }

            if (mWayfinder != null) {
                mWayfinder.setLocation(mLocation.latitude, mLocation.longitude, mLocationFloor);
                mWayfinder.setDestination(mDestination.latitude, mDestination.longitude, mDestinationFloor);
            }

            updateRoute();
        }
    }

    private void updateRoute() {
        if (mLocation == null || mDestination == null || mWayfinder == null) {
            return;
        }
        Log.d(TAG, "Updating the wayfinding route");

        mCurrentRoute = mWayfinder.getRoute();
        if (mCurrentRoute == null || mCurrentRoute.length == 0) {
            // Wrong credentials or invalid wayfinding graph
            return;
        }
        if (mPath != null) {
            // Remove old path if any
            clearOldPath();
        }
        visualizeRoute(mCurrentRoute);
    }


    /**
     * Clear the visualizations for the wayfinding paths
     */
    private void clearOldPath() {
        mPath.remove();
        mPathCurrent.remove();
    }

    /**
     * Visualize the IndoorAtlas Wayfinding path on top of the Google Maps.
     * @param legs Array of IARoutingLeg objects returned from IAWayfinder.getRoute()
     */
    private void visualizeRoute(IARoutingLeg[] legs) {
        // optCurrent will contain the wayfinding path in the current floor and opt will contain the
        // whole path, including parts in other floors.
        PolylineOptions opt = new PolylineOptions();
        PolylineOptions optCurrent = new PolylineOptions();

        for (IARoutingLeg leg : legs) {
            opt.add(new LatLng(leg.getBegin().getLatitude(), leg.getBegin().getLongitude()));
            if (leg.getBegin().getFloor() == mLocationFloor && leg.getEnd().getFloor() == mLocationFloor) {
                optCurrent.add(
                        new LatLng(leg.getBegin().getLatitude(), leg.getBegin().getLongitude()));
                optCurrent.add(
                        new LatLng(leg.getEnd().getLatitude(), leg.getEnd().getLongitude()));
            }
        }
        optCurrent.color(Color.RED);
        if (legs.length > 0) {
            IARoutingLeg leg = legs[legs.length-1];
            opt.add(new LatLng(leg.getEnd().getLatitude(), leg.getEnd().getLongitude()));
        }
        // Here wayfinding path in different floor than current location is visualized in blue and
        // path in current floor is visualized in red
        mPath = mMap.addPolyline(opt);
        mPathCurrent = mMap.addPolyline(optCurrent);

        IARoutingLeg new_legs[] = new IARoutingLeg[legs.length];
        int j = 0;

        for (int i = 0; i < legs.length; i++) {
            if (legs[i].getLength() >= 2.0) {
                double new_dir_i = 90 - legs[i].getDirection() * 180.0 / 3.14159265;
                new_dir_i = Math.round(new_dir_i);
                new_legs[j] = new IARoutingLeg(null, null, legs[i].getLength(), new_dir_i, null);
                j++;
            }
        }

        if (new_legs.length > 0) {

            textView.setText("First Leg: Direction: " + new_legs[0].getDirection() + " Length: " + new_legs[0].getLength());

            // send wayfinding directions to server
            for (int i = 0; i < legs.length; i++) {
                Log.d("Wayfinding_Legs", "Original Direction Deg90-dir: " + (90 - (legs[i].getDirection() * 180.0 / 3.14159265))  + " Length: " + legs[i].getLength());
            }

            ArrayList<StringPair> jsonList = new ArrayList<>();
            jsonList.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_WAYFINDING));
            for (int i = 0; i < j; i++) {
                Log.d("Wayfinding_Legs", "New Direction: " + new_legs[i].getDirection() + " Length: " + new_legs[i].getLength());

                if (i < 5) {
                    jsonList.add(new StringPair(ServerLink.DIR[i], Double.toString(new_legs[i].getDirection())));
                    jsonList.add(new StringPair(ServerLink.LEN[i], Double.toString(new_legs[i].getLength())));
                }
            }
            for (; j < 5; j++) {
                jsonList.add(new StringPair(ServerLink.DIR[j], "0"));
                jsonList.add(new StringPair(ServerLink.LEN[j], "0"));
            }
            myserver.request(jsonList);

            if (current_state == STATE_ENDED) {
                ArrayList<StringPair> jsonList1 = new ArrayList<>();
                jsonList.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_PATH_FINISHED));
                myserver.request(jsonList1);
            }
        }
    }

    /**
     * Sets bitmap of floor plan as ground overlay on Google Maps
     */
    public void setupGroundOverlay(IAFloorPlan floorPlan, Bitmap bitmap) {

        if (floorPlan == null || bitmap == null) {
            return;
        }

        if (mGroundOverlay != null) {
            mGroundOverlay.remove();
        }

        if (mMap != null) {
            BitmapDescriptor bitmapDescriptor = BitmapDescriptorFactory.fromBitmap(bitmap);
            IALatLng iaLatLng = floorPlan.getCenter();
            LatLng center = new LatLng(iaLatLng.latitude, iaLatLng.longitude);
            GroundOverlayOptions fpOverlay = new GroundOverlayOptions()
                    .image(bitmapDescriptor)
                    .zIndex(0.0f)
                    .position(center, floorPlan.getWidthMeters(), floorPlan.getHeightMeters())
                    .bearing(floorPlan.getBearing());

            mGroundOverlay = mMap.addGroundOverlay(fpOverlay);
        }
    }

    /**
     * Load "wayfinding_graph.json" from raw resources folder of the app module
     * @return
     */
    private String loadGraphJSON() {
        try {
            Resources res = getResources();
            int resourceIdentifier = res.getIdentifier("wayfinding_graph", "raw", this.getPackageName());
            InputStream in_s = res.openRawResource(resourceIdentifier);

            byte[] b = new byte[in_s.available()];
            in_s.read(b);
            return new String(b);
        } catch (Exception e) {
            Log.e(TAG, "Could not find wayfinding_graph.json from raw resources folder");
            return null;
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(runnableCode);
        mMapView.onDestroy();
        if (mWayfinder != null) {
            mWayfinder.close();
        }
        atlas.destroy();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mMapView.onPause();
        atlas.pause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        atlas.resume();
        mMapView.onResume();
    }

    @Override
    protected void onStart() {
        super.onStart();
        mMapView.onStart();
    }

    @Override
    protected void onStop() {
        super.onStop();
        mMapView.onStop();
    }

    public void clickEmergency(View view) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_EMERGENCY));
        jsonlist.add(new StringPair(ServerLink.NAME, name));
        myserver.request(jsonlist);

        findViewById(R.id.button2).setBackgroundColor(Color.RED);
    }

}
