package com.example.guardianangelroverv3;


import android.support.v7.app.AppCompatActivity;

/**
 * Created by rachelclark on 3/10/18.
 */

public class CurrentActivity {

    ////
    // 1 = MainActivity
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
