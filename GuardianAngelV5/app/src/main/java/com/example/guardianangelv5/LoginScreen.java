package com.example.guardianangelv5;

import android.Manifest;
import android.content.Intent;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;

public class LoginScreen extends AppCompatActivity {

    // message for Intent
    public static final String EXTRA_MESSAGE = "com.example.guardianangelv4.MESSAGE";

    // server comms info
    private static ServerLink myserver;

    // IndoorAtlas
    private static IndoorAtlas atlas;

    // floor plan ID
    //private static final String FLOOR_PLAN_ID = "f97a76f2-ffd1-4038-b7e6-870dba48c8b5"; // second floor
    private static final String FLOOR_PLAN_ID = "d039ddaa-6813-4412-83dd-705d5904f352"; // third floor

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login_screen);

        // set CurrentActivity
        CurrentActivity.getInstance().setCurrentActivity(1, this);

        // set up server link
        myserver = ServerLink.getInstance();

        // prompt for IP address
        Helper.createIPDialog(myserver);

        // Request permissions for IndoorAtlas
        String[] neededPermissions = {
                android.Manifest.permission.CHANGE_WIFI_STATE,
                android.Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.ACCESS_COARSE_LOCATION
        };
        int CODE_PERMISSIONS = 0;
        ActivityCompat.requestPermissions( this, neededPermissions, CODE_PERMISSIONS);

        // Initialize IndoorAtlas
        atlas = IndoorAtlas.getInstance();
        atlas.fetchFloorPlan(FLOOR_PLAN_ID);
    }

    // Called when user presses submit
    public void submitName(View view) {
        Intent intent = new Intent(this, RequestScreen.class);
        EditText editText = findViewById(R.id.editText);
        String message = editText.getText().toString();
        intent.putExtra(EXTRA_MESSAGE, message);
        startActivity(intent);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        atlas.destroy();
    }

    @Override
    protected void onPause() {
        super.onPause();
        atlas.pause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        atlas.resume();
    }

}
