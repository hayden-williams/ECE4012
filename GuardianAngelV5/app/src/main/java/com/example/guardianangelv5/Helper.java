package com.example.guardianangelv5;

import android.app.Activity;
import android.app.Dialog;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.indooratlas.android.sdk.resources.IAFloorPlan;
import com.indooratlas.android.sdk.resources.IALatLng;
import com.indooratlas.android.wayfinding.IARoutingLeg;
import com.indooratlas.android.wayfinding.IAWayfinder;

import java.io.InputStream;
import java.util.ArrayList;

public class Helper {

    public static final float HUE_IABLUE = 200.0f;

    public static void createIPDialog(final ServerLink myserver) {
        final Dialog dialog = new Dialog(CurrentActivity.getInstance().getCurrentActivity());
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
    }

    public static Marker updateMap(GoogleMap mMap, Marker mMarker, IndoorAtlas atlas) {
        if (mMap == null) {
            // location received before map is initialized, ignoring update here
            return mMarker;
        }

        Log.d("Map", "Updating map");

        LatLng mLocation = new LatLng(atlas.getLat(), atlas.getLon());
        if (mMarker == null) {
            if (mMap != null) {
                mMarker = mMap.addMarker(new MarkerOptions().position(mLocation)
                        .icon(BitmapDescriptorFactory.defaultMarker(HUE_IABLUE))
                        .title("Your Location"));
                mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(mLocation, 17.0f));
                return mMarker;
            }
        } else {
            mMarker.setPosition(mLocation);
            return mMarker;
        }
        return mMarker;
    }

    /**
     * Sets bitmap of floor plan as ground overlay on Google Maps
     */
    public static GroundOverlay setupGroundOverlay(IAFloorPlan floorPlan, Bitmap bitmap,
                                                   GroundOverlay mGroundOverlay, GoogleMap mMap) {

        if (floorPlan == null || bitmap == null) {
            return mGroundOverlay;
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
            return mGroundOverlay;
        }
        return mGroundOverlay;
    }

    public static void updateServerLocationUser(String user_name,
                                                IndoorAtlas atlas, ServerLink myserver) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_USER));
        jsonlist.add(new StringPair(ServerLink.NAME, user_name));
        jsonlist.add(new StringPair(ServerLink.LAT, Double.toString(atlas.getLat())));
        jsonlist.add(new StringPair(ServerLink.LON, Double.toString(atlas.getLon())));
        myserver.request(jsonlist);
    }

    public static void createPathFromHome(IndoorAtlas atlas, ServerLink myserver) {
        createPath(atlas, myserver, IndoorAtlas.home_lat, IndoorAtlas.home_lon, atlas.lat, atlas.lon);
    }

    public static void createPathToHome(IndoorAtlas atlas, ServerLink myserver) {
        createPath(atlas, myserver, atlas.lat, atlas.lon, IndoorAtlas.home_lat, IndoorAtlas.home_lon);
    }

    private static void createPath(IndoorAtlas atlas, ServerLink myserver, double start_lat,
                                  double start_lon, double end_lat, double end_lon) {
        IAWayfinder mWayfinder;

        // load JSON wayfinding graph
        String graphJSON = loadGraphJSON();
        if (graphJSON == null) {
            Log.d("Load JSON Graph", "Could not find wayfinding_graph.json from raw " +
                    "resources folder. Cannot do wayfinding.");
        } else {
            mWayfinder = IAWayfinder.create(CurrentActivity.getInstance().getCurrentActivity(), graphJSON);
            mWayfinder.setLocation(start_lat, start_lon, 2);
            mWayfinder.setDestination(end_lat, end_lon, 2);

            IARoutingLeg[] legs = mWayfinder.getRoute();
            if (legs == null || legs.length == 0) {
                // Wrong credentials or invalid wayfinding graph
                return;
            }

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

                // send wayfinding directions to server
                ArrayList<StringPair> jsonList = new ArrayList<>();
                jsonList.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_WAYFINDING));
                for (int i = 0; i < j; i++) {
                    Log.d("Wayfinding_Legs", "Direction: " + new_legs[i].getDirection() + " Length: " + new_legs[i].getLength());

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
            }
        }
    }

    /**
     * Load "wayfinding_graph.json" from raw resources folder of the app module
     * @return
     */
    private static String loadGraphJSON() {
        try {
            Resources res = CurrentActivity.getInstance().getCurrentActivity().getResources();
            int resourceIdentifier = res.getIdentifier("wayfinding_graph", "raw", CurrentActivity.getInstance().getCurrentActivity().getPackageName());
            InputStream in_s = res.openRawResource(resourceIdentifier);

            byte[] b = new byte[in_s.available()];
            in_s.read(b);
            return new String(b);
        } catch (Exception e) {
            Log.e("Load JSON Graph", "Could not find wayfinding_graph.json from raw resources folder");
            return null;
        }
    }

}
