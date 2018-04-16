package com.example.guardianangelroverv3;

import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;

import com.google.android.gms.maps.model.GroundOverlay;
import com.indooratlas.android.sdk.IALocation;
import com.indooratlas.android.sdk.IALocationListener;
import com.indooratlas.android.sdk.IALocationManager;
import com.indooratlas.android.sdk.IALocationRequest;
import com.indooratlas.android.sdk.IARegion;
import com.indooratlas.android.sdk.resources.IAFloorPlan;
import com.indooratlas.android.sdk.resources.IAResourceManager;
import com.indooratlas.android.sdk.resources.IAResult;
import com.indooratlas.android.sdk.resources.IAResultCallback;
import com.indooratlas.android.sdk.resources.IATask;
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
    // used to decide when bitmap should be downscaled
    private static final int MAX_DIMENSION = 2048;
    public IAFloorPlan floorPlan_saved = null;
    public Bitmap bitmap_saved = null;

    private String TAG = "IndoorAtlas"; // for log

    public double lat = 33.77596903699503;
    public double lon = -84.39690195434368;
    private int floor = 2;
    private double bearing = 0;

    private double bearing_offset = -10.0;

    private boolean first_location_set = false;

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
                    /*if (!first_location_set) {
                        Log.d(TAG, "Latitude: " + location.getLatitude());
                        lat = location.getLatitude();
                        Log.d(TAG, "Longitude: " + location.getLongitude());
                        lon = location.getLongitude();
                        Log.d(TAG, "Floor number: " + location.getFloorLevel());
                        floor = location.getFloorLevel();
                        Log.d(TAG, "Bearing: " + location.getBearing() + bearing_offset);
                        bearing = location.getBearing() + bearing_offset;

                        //((MainActivity)app).updateServer();
                        ((MainActivity) app).updateMap();
                    }
                    first_location_set = true;*/
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

    public double getBearing() { return bearing; }

    public void destroy() {
        mIALocationManager.destroy();
    }

    public void pause() {
        if (mIALocationManager != null) {
            mIALocationManager.removeLocationUpdates(mIALocationListener);
            mIALocationManager.registerRegionListener(mRegionListener);
        }
    }

    public void resume() {
        mIALocationManager.requestLocationUpdates(IALocationRequest.create(), mIALocationListener);
        mIALocationManager.registerRegionListener(mRegionListener);
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

                    AppCompatActivity app = CurrentActivity.getInstance().getCurrentActivity();
                    switch (CurrentActivity.getInstance().getCurrentActivityNum()) {
                        case 1:
                            ((MainActivity)app).setupGroundOverlay(floorPlan, bitmap);
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
