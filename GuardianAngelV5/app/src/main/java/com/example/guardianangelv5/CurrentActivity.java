package com.example.guardianangelv5;


import android.support.v7.app.AppCompatActivity;

/**
 * Created by rachelclark on 3/10/18.
 */

public class CurrentActivity {

    ////
    // 1 = LoginScreen
    // 2 = RequestScreen
    // 3 = WaitScreen
    // 4 = TripScreen
    // 5 = EndScreen
    ////
    private int currentActivity = 0;
    private AppCompatActivity appCompatActivity = null;

    // singleton stuff
    // -------
    private static CurrentActivity mInstance= null;

    public static synchronized CurrentActivity getInstance(){
        if (mInstance == null) {
            mInstance = new CurrentActivity();
        }
        return mInstance;
    }
    // ---------

    public void setCurrentActivity(int i, AppCompatActivity app) {
        currentActivity = i;
        appCompatActivity = app;
    }

    public int getCurrentActivityNum() {
        return currentActivity;
    }

    public AppCompatActivity getCurrentActivity() {
        return appCompatActivity;
    }
}
