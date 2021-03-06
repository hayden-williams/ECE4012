package com.example.guardianangelroverv4;

import android.app.Dialog;
import android.graphics.Color;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity {

    // threading
    final Handler handler = new Handler();
    private Runnable runnableCode;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        CurrentActivity.getInstance().setCurrentActivity(0, this);

        createIPDialog(ServerLink.getInstance());

        /*runnableCode = new Runnable() {
            @Override
            public void run() {
                Log.d("Handlers", "Called on main thread");
                // check server for emergency
                handler.postDelayed(this, 5000);
            }
        };
        handler.post(runnableCode);*/
    }

    public void clickEmergency(View view) {
        ArrayList<StringPair> jsonlist = new ArrayList<>();
        jsonlist.add(new StringPair(ServerLink.MESSAGE_TYPE, ServerLink.MESSAGE_TYPE_EMERGENCY));
        jsonlist.add(new StringPair(ServerLink.NAME, "ROVER"));
        ServerLink.getInstance().request(jsonlist);

        findViewById(R.id.button).setBackgroundColor(Color.RED);


    }

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
}
