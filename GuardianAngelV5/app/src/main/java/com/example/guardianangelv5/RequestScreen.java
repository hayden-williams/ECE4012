package com.example.guardianangelv5;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;

public class RequestScreen extends AppCompatActivity implements OnMapReadyCallback {

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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_request_screen);

        // set CurrentActivity
        CurrentActivity.getInstance().setCurrentActivity(2, this);

        // Get the Intent that started this activity and extract the string
        Intent intent = getIntent();
        user_name = intent.getStringExtra(LoginScreen.EXTRA_MESSAGE);

        myserver = ServerLink.getInstance();
        atlas = IndoorAtlas.getInstance();
        atlas.fetchFloorPlan("f97a76f2-ffd1-4038-b7e6-870dba48c8b5");

        mMapView = findViewById(R.id.mapView);
        mMapView.onCreate(savedInstanceState);
        mMapView.getMapAsync(this);

        mGroundOverlay = Helper.setupGroundOverlay(atlas.floorPlan_saved, atlas.bitmap_saved, mGroundOverlay, mMap);
    }

    public void clickRequest(View view) {
        Intent intent = new Intent(this, WaitScreen.class);
        intent.putExtra(LoginScreen.EXTRA_MESSAGE, user_name);
        startActivity(intent);
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;
        mMap.getUiSettings().setMyLocationButtonEnabled(false);
        mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {
                atlas.lat = latLng.latitude;
                atlas.lon = latLng.longitude;
                mMarker = Helper.updateMap(mMap, mMarker, atlas);
            }
        });
        if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            String[] permissions = new String[2];
            permissions[0] = android.Manifest.permission.ACCESS_FINE_LOCATION;
            permissions[1] = Manifest.permission.ACCESS_COARSE_LOCATION;
            ActivityCompat.requestPermissions(this, permissions, 0);
            return;
        }
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
        mMapView.onDestroy();
        atlas.destroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
        mMapView.onLowMemory();
    }

    public void setMarker(Marker m) {
        mMarker = m;
    }

    public Marker getMarker() {
        return mMarker;
    }

    public GoogleMap getMap() {
        return mMap;
    }

    public IndoorAtlas getAtlas() {
        return atlas;
    }

    public void setOverlay(GroundOverlay g) {
        mGroundOverlay = g;
    }

    public GroundOverlay getOverlay() {
        return mGroundOverlay;
    }
}
