package com.example.guardianangelv5;

import android.app.Dialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Handler;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.Marker;

import java.util.ArrayList;

public class TripScreen extends AppCompatActivity implements OnMapReadyCallback {

    // debugging
    private static final String TAG = "TripScreen";

    // user_name from LoginScreen
    private String user_name;

    // comms and location
    private static ServerLink myserver;
    private static IndoorAtlas atlas;

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
        setContentView(R.layout.activity_trip_screen);

        // set CurrentActivity
        CurrentActivity.getInstance().setCurrentActivity(4, this);

        // Get the Intent that started this activity and extract the string
        Intent intent = getIntent();
        user_name = intent.getStringExtra(LoginScreen.EXTRA_MESSAGE);

        myserver = ServerLink.getInstance();
        atlas = IndoorAtlas.getInstance();
        atlas.fetchFloorPlan("f97a76f2-ffd1-4038-b7e6-870dba48c8b5");

        mMapView = findViewById(R.id.mapView3);
        mMapView.onCreate(savedInstanceState);
        mMapView.getMapAsync(this);

        Helper.setupGroundOverlay(atlas.floorPlan_saved, atlas.bitmap_saved, mGroundOverlay, mMap);

        // emergency button - must hold down
        findViewById(R.id.button6).setOnTouchListener(new View.OnTouchListener() {
            long emergencyPressTime = 0;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    emergencyPressTime = event.getEventTime();
                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    if (event.getEventTime() - emergencyPressTime > 4000) {
                        Log.d(TAG, "Emergency triggered");
                        clickEmergency(v);
                    }
                }
                return true;
            }
        });

        runnableCode = new Runnable() {
            @Override
            public void run() {
                Log.d("Handlers", "Called on main thread");
                Helper.updateServerLocationUser(user_name, atlas, myserver);
                handler.postDelayed(this, 5000);
            }
        };
        handler.post(runnableCode);

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

    public void clickEnd(View view) {
        handler.removeCallbacks(runnableCode);

        Intent intent = new Intent(this, EndScreen.class);
        intent.putExtra(LoginScreen.EXTRA_MESSAGE, user_name);
        startActivity(intent);
    }

    public void clickEmergency(View view) {
        final Dialog dialog = new Dialog(this);
        dialog.setContentView(R.layout.emergency_dialog);
        dialog.setTitle("Emergency");

        Button button = dialog.findViewById(R.id.embutton);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                dialog.dismiss();
            }
        });

        dialog.show();

        // Hide after some seconds
        final Handler handler  = new Handler();
        final Runnable runnable = new Runnable() {
            @Override
            public void run() {
                if (dialog.isShowing()) {
                    // dialog still showing after timeout
                    // trigger emergency
                    ArrayList<StringPair> jsonlist = new ArrayList<>();
                    jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_EMERGENCY));
                    jsonlist.add(new StringPair(ServerLink.NAME, user_name));
                    myserver.request(jsonlist);

                    findViewById(R.id.button6).setBackgroundColor(Color.RED);

                    dialog.dismiss();
                }
            }
        };
        dialog.setOnDismissListener(new DialogInterface.OnDismissListener() {
            @Override
            public void onDismiss(DialogInterface dialog) {
                handler.removeCallbacks(runnable);
            }
        });
        handler.postDelayed(runnable, 5000);
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
