package com.example.guardianangelv4;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

public class EndScreen extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_end_screen);

        CurrentActivity.getInstance().setCurrentActivity(5, this);
        IndoorAtlas.getInstance().pause();
        IndoorAtlas.getInstance().destroy();
    }

}
