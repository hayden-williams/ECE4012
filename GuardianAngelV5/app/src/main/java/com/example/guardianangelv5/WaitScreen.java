package com.example.guardianangelv5;

import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Handler;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.jakewharton.processphoenix.ProcessPhoenix;

import java.util.ArrayList;

public class WaitScreen extends AppCompatActivity implements OnMapReadyCallback {

    // server comms and location
    private static ServerLink myserver;
    private static IndoorAtlas atlas;

    // name from LoginScreen
    private String user_name;

    // mapping
    private GoogleMap mMap;
    private Marker mMarker;
    private MapView mMapView;
    private GroundOverlay mGroundOverlay = null;

    // threading
    final Handler handler = new Handler();
    private Runnable runnableCode;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_wait_screen);

        // set CurrentActivity
        CurrentActivity.getInstance().setCurrentActivity(3, this);

        // Get the Intent that started this activity and extract the string
        Intent intent = getIntent();
        user_name = intent.getStringExtra(LoginScreen.EXTRA_MESSAGE);

        myserver = ServerLink.getInstance();
        atlas = IndoorAtlas.getInstance();

        mMapView = findViewById(R.id.mapView2);
        mMapView.onCreate(savedInstanceState);
        mMapView.getMapAsync(this);

        runnableCode = new Runnable() {
            @Override
            public void run() {
                Log.d("Handlers", "Called on main thread");
                Helper.updateServerLocationUser(user_name, atlas, myserver);
                handler.postDelayed(this, 5000);
            }
        };
        handler.post(runnableCode);

        Helper.updateServerLocationUser(user_name, atlas, myserver);
        Helper.createPathFromHome(atlas, myserver);
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;
        mMap.getUiSettings().setMyLocationButtonEnabled(false);
        mGroundOverlay = Helper.setupGroundOverlay(atlas.floorPlan_saved, atlas.bitmap_saved, mGroundOverlay, mMap);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(atlas.lat, atlas.lon), 17.0f));
        if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            String[] permissions = new String[2];
            permissions[0] = android.Manifest.permission.ACCESS_FINE_LOCATION;
            permissions[1] = android.Manifest.permission.ACCESS_COARSE_LOCATION;
            ActivityCompat.requestPermissions(this, permissions, 0);
            return;
        }
        mMarker = Helper.updateMap(mMap, mMarker, atlas);
    }

    public void clickArrived(View view) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_ROVER_ARRIVED));
        jsonlist.add(new StringPair(ServerLink.NAME, user_name));
        myserver.request(jsonlist);

        handler.removeCallbacks(runnableCode);

        Intent intent = new Intent(this, TripScreen.class);
        intent.putExtra(LoginScreen.EXTRA_MESSAGE, user_name);
        startActivity(intent);
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

    public void setOverlay(GroundOverlay g) {
        mGroundOverlay = g;
    }

    public GroundOverlay getOverlay() {
        return mGroundOverlay;
    }

    public GoogleMap getMap() {
        return mMap;
    }

    public ServerLink getServer() {
        return myserver;
    }

    public IndoorAtlas getAtlas() {
        return atlas;
    }

    public String getUserName() {
        return user_name;
    }
}
