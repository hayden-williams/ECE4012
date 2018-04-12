package com.example.guardianangelv4;

import android.Manifest;
import android.app.Dialog;
import android.content.Intent;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

public class LoginScreen extends AppCompatActivity {

    public static final String EXTRA_MESSAGE = "com.example.guardianangelv4.MESSAGE";

    // server comms info
    private static ServerLink myserver;

    // IndoorAtlas
    private static IndoorAtlas atlas;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login_screen);

        final Dialog dialog = new Dialog(LoginScreen.this);
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

        CurrentActivity.getInstance().setCurrentActivity(1, this);

        // set up server link
        myserver = ServerLink.getInstance();

        // Request permissions for IndoorAtlas
        String[] neededPermissions = {
                Manifest.permission.CHANGE_WIFI_STATE,
                Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.ACCESS_COARSE_LOCATION
        };
        int CODE_PERMISSIONS = 0;
        ActivityCompat.requestPermissions( this, neededPermissions, CODE_PERMISSIONS);

        // Initialize IndoorAtlas
        atlas = IndoorAtlas.getInstance();

    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        //Handle if any of the permissions are denied, in grantResults
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
