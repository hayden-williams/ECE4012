package com.example.guardianangelv4;

import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Handler;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

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
import com.indooratlas.android.sdk.resources.IAFloorPlan;
import com.indooratlas.android.sdk.resources.IALatLng;

import java.util.ArrayList;

public class TripScreen extends AppCompatActivity implements OnMapReadyCallback {

    // debugging
    private static final String TAG = "TripScreen";

    private String user_name;

    private static ServerLink myserver;
    private static IndoorAtlas atlas;

    private GoogleMap mMap;
    private Marker mMarker;
    private Marker mMarkerOther;
    private MapView mMapView;
    private GroundOverlay mGroundOverlay = null;

    private LatLng mLocation;

    private static final float HUE_IABLUE = 200.0f;

    final Handler handler = new Handler();
    private Runnable runnableCode;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_trip_screen);

        CurrentActivity.getInstance().setCurrentActivity(4, this);

        // Get the Intent that started this activity and extract the string
        Intent intent = getIntent();
        user_name = intent.getStringExtra(LoginScreen.EXTRA_MESSAGE);

        myserver = ServerLink.getInstance();
        atlas = IndoorAtlas.getInstance();

        mMapView = findViewById(R.id.mapView3);
        mMapView.onCreate(savedInstanceState);
        mMapView.getMapAsync(this);

        setupGroundOverlay(atlas.floorPlan_saved, atlas.bitmap_saved);

        runnableCode = new Runnable() {
            @Override
            public void run() {
                Log.d("Handlers", "Called on main thread");
                updateServer();
                handler.postDelayed(this, 5000);
            }
        };
        handler.post(runnableCode);
    }

    public void updateServer() {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_USER));
        jsonlist.add(new StringPair(ServerLink.NAME, user_name));
        jsonlist.add(new StringPair(ServerLink.LAT, Double.toString(atlas.getLat())));
        jsonlist.add(new StringPair(ServerLink.LON, Double.toString(atlas.getLon())));
        jsonlist.add(new StringPair(ServerLink.FLOOR, Integer.toString(atlas.getFloor())));
        myserver.request(jsonlist);
    }

    public void clickEnd(View view) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_END_TRIP));
        jsonlist.add(new StringPair(ServerLink.NAME, user_name));
        myserver.request(jsonlist);

        handler.removeCallbacks(runnableCode);

        Intent intent = new Intent(this, EndScreen.class);
        startActivity(intent);
    }

    public void clickEmergency(View view) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_EMERGENCY));
        jsonlist.add(new StringPair(ServerLink.NAME, user_name));
        myserver.request(jsonlist);

        findViewById(R.id.button6).setBackgroundColor(Color.RED);
    }

    @Override
    public void onResume() {
        super.onResume();
        mMapView.onResume();
        atlas.resume();
    }


    @Override
    public void onPause() {
        super.onPause();
        mMapView.onPause();
        atlas.pause();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(runnableCode);
        mMapView.onDestroy();
        atlas.destroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
        mMapView.onLowMemory();
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;
        mMap.getUiSettings().setMyLocationButtonEnabled(false);
        if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            String[] permissions = new String[2];
            permissions[0] = android.Manifest.permission.ACCESS_FINE_LOCATION;
            permissions[1] = android.Manifest.permission.ACCESS_COARSE_LOCATION;
            ActivityCompat.requestPermissions(this, permissions, 0);
            return;
        }
    }

    public void updateMap() {
        if (mMap == null) {
            // location received before map is initialized, ignoring update here
            return;
        }

        Log.d("Map", "Updating map");

        mLocation = new LatLng(atlas.getLat(), atlas.getLon());
        if (mMarker == null) {
            if (mMap != null) {
                mMarker = mMap.addMarker(new MarkerOptions().position(mLocation)
                        .icon(BitmapDescriptorFactory.defaultMarker(HUE_IABLUE))
                        .title("Your Location"));
                mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(mLocation, 17.0f));
            }
        } else {
            mMarker.setPosition(mLocation);
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

}
