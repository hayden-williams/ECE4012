package com.example.guardianangelv4;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;

import com.google.android.gms.maps.model.BitmapDescriptor;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.GroundOverlay;
import com.google.android.gms.maps.model.GroundOverlayOptions;
import com.google.android.gms.maps.model.LatLng;
import com.indooratlas.android.sdk.IALocation;
import com.indooratlas.android.sdk.IALocationListener;
import com.indooratlas.android.sdk.IALocationManager;
import com.indooratlas.android.sdk.IALocationRequest;
import com.indooratlas.android.sdk.IARegion;
import com.indooratlas.android.sdk.resources.IAFloorPlan;
import com.indooratlas.android.sdk.resources.IALatLng;
import com.indooratlas.android.sdk.resources.IAResourceManager;
import com.indooratlas.android.sdk.resources.IAResult;
import com.indooratlas.android.sdk.resources.IAResultCallback;
import com.indooratlas.android.sdk.resources.IATask;

import java.lang.annotation.Annotation;
import java.lang.annotation.ElementType;

import com.squareup.picasso.Picasso;
import com.squareup.picasso.RequestCreator;
import com.squareup.picasso.Target;

/**
 * Created by rachelclark on 2/27/18.
 *
 * Class to make singleton IndoorAtlas variable to get location.
 */

public class IndoorAtlas {

    // singleton stuff
    // -------
    private static IndoorAtlas mInstance= null;

    public static synchronized IndoorAtlas getInstance(){
        if (mInstance == null) {
            mInstance = new IndoorAtlas();
        }
        return mInstance;
    }

    // ---------

    private IALocationManager mIALocationManager;
    private IARegion mOverlayFloorPlan = null;
    private GroundOverlay mGroundOverlay = null;
    private IAResourceManager mResourceManager = null;
    private IATask<IAFloorPlan> mFetchFloorPlanTask;

    private Target mLoadTarget;
    /* used to decide when bitmap should be downscaled */
    private static final int MAX_DIMENSION = 2048;
    public IAFloorPlan floorPlan_saved = null;
    public Bitmap bitmap_saved = null;

    private String TAG = "IndoorAtlas"; // for log

    public double lat = 0;
    public double lon = 0;
    public int floor = 0;

    private boolean first_location_set[] = {false, false, false, false, false};

    protected IndoorAtlas() {
        // Initialize IndoorAtlas
        mIALocationManager = IALocationManager.create(CurrentActivity.getInstance().getCurrentActivity());
        mResourceManager = IAResourceManager.create(CurrentActivity.getInstance().getCurrentActivity());
        mIALocationManager.requestLocationUpdates(IALocationRequest.create(), mIALocationListener);
        mIALocationManager.registerRegionListener(mRegionListener);
    }

    private IALocationListener mIALocationListener = new IALocationListener() {
        // Called when the location has changed.
        @Override
        public void onLocationChanged(IALocation location) {
                AppCompatActivity app = CurrentActivity.getInstance().getCurrentActivity();
                switch (CurrentActivity.getInstance().getCurrentActivityNum()) {
                    case 1:
                        break;
                    case 2:
                        if (!first_location_set[1]) {
                            Log.d(TAG, "Latitude: " + location.getLatitude());
                            lat = location.getLatitude();
                            Log.d(TAG, "Longitude: " + location.getLongitude());
                            lon = location.getLongitude();
                            Log.d(TAG, "Floor number: " + location.getFloorLevel());
                            floor = location.getFloorLevel();
                            ((RequestScreen) app).updateMap();
                        }
                        first_location_set[1] = true;
                        break;
                    case 3:
                        //((WaitScreen)app).updateServer();
                        //((WaitScreen) app).updateMap();
                        break;
                    case 4:
                        Log.d(TAG, "Latitude: " + location.getLatitude());
                        lat = location.getLatitude();
                        Log.d(TAG, "Longitude: " + location.getLongitude());
                        lon = location.getLongitude();
                        Log.d(TAG, "Floor number: " + location.getFloorLevel());
                        floor = location.getFloorLevel();
                        //((TripScreen)app).updateServer();
                        ((TripScreen) app).updateMap();
                        break;
                    case 5:
                        if (!first_location_set[4]) {
                            Log.d(TAG, "Latitude: " + location.getLatitude());
                            lat = location.getLatitude();
                            Log.d(TAG, "Longitude: " + location.getLongitude());
                            lon = location.getLongitude();
                            Log.d(TAG, "Floor number: " + location.getFloorLevel());
                            floor = location.getFloorLevel();
                            ((EndScreen) app).updateMap();
                        }
                        first_location_set[4] = true;
                        break;
                }
        }

        @Override
        public void onStatusChanged(String s, int i, Bundle bundle) {

        }
    };

    /** Minimal floor detection example */
    private IARegion.Listener mRegionListener = new IARegion.Listener() {
        //IARegion mCurrentFloorPlan = null;

        @Override
        public void onEnterRegion(IARegion region) {
            if (region.getType() == IARegion.TYPE_FLOOR_PLAN) {
                Log.d(TAG, "Entered " + region.getName());
                Log.d(TAG, "floor plan ID: " + region.getId());
                //mCurrentFloorPlan = region;

                final String newId = region.getId();
                // Are we entering a new floor plan or coming back the floor plan we just left?
                if (mGroundOverlay == null || !region.equals(mOverlayFloorPlan)) {
                    //mCameraPositionNeedsUpdating = true; // entering new fp, need to move camera
                    if (mGroundOverlay != null) {
                        mGroundOverlay.remove();
                        mGroundOverlay = null;
                    }
                    mOverlayFloorPlan = region; // overlay will be this (unless error in loading)
                    fetchFloorPlan(newId);
                } else {
                    mGroundOverlay.setTransparency(0.0f);
                }

            }

        }

        @Override
        public void onExitRegion(IARegion region) {
            if (mGroundOverlay != null) {
                // Indicate we left this floor plan but leave it there for reference
                // If we enter another floor plan, this one will be removed and another one loaded
                mGroundOverlay.setTransparency(0.5f);
            }
        }
    };

    public double getLat() {
        return lat;
    }

    public double getLon() {
        return lon;
    }

    public int getFloor() {
        return floor;
    }

    public void destroy() {
        mIALocationManager.destroy();
    }

    public void pause() {
        if (mIALocationManager != null) {
            mIALocationManager.removeLocationUpdates(mIALocationListener);
        }
    }

    public void resume() {
        mIALocationManager.requestLocationUpdates(IALocationRequest.create(), mIALocationListener);
    }


    /**
     * Fetches floor plan data from IndoorAtlas server.
     */
    private void fetchFloorPlan(String id) {

        if (mResourceManager == null) {
            return;
        }

        // if there is already running task, cancel it
        cancelPendingNetworkCalls();

        final IATask<IAFloorPlan> task = mResourceManager.fetchFloorPlanWithId(id);

        task.setCallback(new IAResultCallback<IAFloorPlan>() {

            @Override
            public void onResult(IAResult<IAFloorPlan> result) {

                if (result.isSuccess() && result.getResult() != null) {
                    Log.d("FloorPlanFetch", "success");
                    // retrieve bitmap for this floor plan metadata
                    fetchFloorPlanBitmap(result.getResult());
                } else {
                    // ignore errors if this task was already canceled
                    if (!task.isCancelled()) {
                        // do something with error
                        //showInfo("Loading floor plan failed: " + result.getError());
                        Log.d("FloorPlanFetch", "failed");
                        mOverlayFloorPlan = null;
                    }
                }
            }
        }, Looper.getMainLooper()); // deliver callbacks using main looper

        // keep reference to task so that it can be canceled if needed
        mFetchFloorPlanTask = task;

    }

    /**
     * Download floor plan using Picasso library.
     */
    private void fetchFloorPlanBitmap(final IAFloorPlan floorPlan) {

        final String url = floorPlan.getUrl();

        if (mLoadTarget == null) {
            mLoadTarget = new Target() {
                @Override
                public void onBitmapLoaded(Bitmap bitmap, Picasso.LoadedFrom from) {
                    Log.d(TAG, "onBitmap loaded with dimensions: " + bitmap.getWidth() + "x"
                            + bitmap.getHeight());

                    floorPlan_saved = floorPlan;
                    bitmap_saved = bitmap;

                    AppCompatActivity app = CurrentActivity.getInstance().getCurrentActivity();
                    switch (CurrentActivity.getInstance().getCurrentActivityNum()) {
                        case 1:
                            break;
                        case 2:
                            ((RequestScreen)app).setupGroundOverlay(floorPlan, bitmap);
                            break;
                        case 3:
                            ((WaitScreen)app).setupGroundOverlay(floorPlan, bitmap);
                            break;
                        case 4:
                            ((TripScreen)app).setupGroundOverlay(floorPlan, bitmap);
                            break;
                        case 5:
                            ((EndScreen)app).setupGroundOverlay(floorPlan, bitmap);
                            break;
                    }
                }

                @Override
                public void onPrepareLoad(Drawable placeHolderDrawable) {
                    // N/A
                }

                @Override
                public void onBitmapFailed(Drawable placeHolderDraweble) {
                    //showInfo("Failed to load bitmap");
                    mOverlayFloorPlan = null;
                }
            };
        }

        RequestCreator request = Picasso.with(CurrentActivity.getInstance().getCurrentActivity()).load(url);

        final int bitmapWidth = floorPlan.getBitmapWidth();
        final int bitmapHeight = floorPlan.getBitmapHeight();

        if (bitmapHeight > MAX_DIMENSION) {
            request.resize(0, MAX_DIMENSION);
        } else if (bitmapWidth > MAX_DIMENSION) {
            request.resize(MAX_DIMENSION, 0);
        }

        request.into(mLoadTarget);
    }

    /**
     * Helper method to cancel current task if any.
     */
    private void cancelPendingNetworkCalls() {
        if (mFetchFloorPlanTask != null && !mFetchFloorPlanTask.isCancelled()) {
            mFetchFloorPlanTask.cancel();
        }
    }

}
