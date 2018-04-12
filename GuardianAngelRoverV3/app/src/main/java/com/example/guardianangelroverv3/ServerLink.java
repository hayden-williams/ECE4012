package com.example.guardianangelroverv3;

import android.app.Activity;
import android.util.Log;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonArrayRequest;
import com.android.volley.toolbox.JsonObjectRequest;
import com.android.volley.toolbox.Volley;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

/**
 * Created by rachelclark on 2/27/18.
 *
 * Class to create singleton ServerLink object to communicate with server.
 */

public class ServerLink {

    // singleton stuff
    // -------
    private static ServerLink mInstance= null;

    public static synchronized ServerLink getInstance(){
        if (mInstance == null) {
            mInstance = new ServerLink();
        }
        return mInstance;
    }

    // ---------

    public static final String MESSAGE_TYPE = "message_type";
    public static final String NAME = "user_name";
    public static final String LAT = "lat";
    public static final String LON = "lon";
    public static final String FLOOR = "floor";
    public static final String DIRECTION = "direction";
    public static final String LENGTH = "length";
    public static final String BEARING = "bearing";

    public static final String MESSAGE_TYPE_ROVER = "0";
    public static final String MESSAGE_TYPE_USER = "1";
    public static final String MESSAGE_TYPE_END_TRIP = "2";
    public static final String MESSAGE_TYPE_EMERGENCY = "3";
    public static final String MESSAGE_TYPE_ROVER_ARRIVED = "4";
    public static final String MESSAGE_TYPE_WAYFINDING = "5";


    private String ip_address = "128.61.114.24";
    private String TAG = "ServerLink";
    private RequestQueue queue;

    public double other_lat = 0;
    public double other_lon = 0;
    public int floor = 0;
    public String user_name = "";
    public boolean emergency = false;

    protected ServerLink() {
        queue = Volley.newRequestQueue(CurrentActivity.getInstance().getCurrentActivity());
    }

    public void setIPAddress(String ip) {
        ip_address = ip;
    }

    // Send JSON data to server
    public void request(List<StringPair> jsonList, Activity act) {
        String url = "http://" + ip_address + ":3000/guardian-angel";

        Log.d(TAG, "Begin sending JSON data");

        JSONObject jsonBody = new JSONObject();
        try {
            for (StringPair listItem : jsonList) {
                jsonBody.put(listItem.getName(), listItem.getValue());
            }
        } catch (JSONException e) {
            Log.d(TAG, "JSONException: " + e.toString());
        }

        JsonObjectRequest request = new JsonObjectRequest(Request.Method.POST, url, jsonBody,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        Log.d(TAG, "Response is: " + response.toString());
                    }
                },
                new Response.ErrorListener() {
                    @Override
                    public void onErrorResponse(VolleyError error) {
                        Log.d(TAG, "Message could not be sent");
                    }
                });
        queue.add(request);
    }

    // Get JSON data from server
    public void getRequest(Activity act, final String message_type, final String provided_name, int num_messages) {
        String url;
        if (provided_name == "") url = "http://" + ip_address + ":3000/guardian-angel?" + MESSAGE_TYPE + "=" + message_type + "&_limit=" + num_messages + "&_sort=id&_order=desc";
        else url = "http://" + ip_address + ":3000/guardian-angel?" + MESSAGE_TYPE + "=" + message_type + "&_" + NAME + "=" + user_name + "&_limit=" + num_messages + "&_sort=id&_order=desc";

        Log.d(TAG, "Begin requesting JSON data");

        JsonArrayRequest request = new JsonArrayRequest(url,
                new Response.Listener<JSONArray>() {
                    @Override
                    public void onResponse(JSONArray response) {
                        Log.d(TAG, "Response is: " + response.toString());
                        if (message_type == MESSAGE_TYPE_USER || message_type == MESSAGE_TYPE_ROVER) {
                            try {
                                other_lat = ((JSONObject) response.get(0)).getDouble(LAT);
                                other_lon = ((JSONObject) response.get(0)).getDouble(LON);
                                floor = ((JSONObject) response.get(0)).getInt(FLOOR);
                                user_name = ((JSONObject) response.get(0)).getString(NAME);
                            } catch (JSONException e) {
                                e.printStackTrace();
                            }
                        } else if (message_type == MESSAGE_TYPE_EMERGENCY) {
                            if (response.length() > 0) {
                                for (int i = 0; i < response.length(); i++) {
                                    try {
                                        String temp_user = ((JSONObject) response.get(i)).getString(NAME);
                                        if (temp_user == provided_name) emergency = true;
                                    } catch (JSONException e) {
                                        e.printStackTrace();
                                    }
                                }
                            } else {
                                emergency = false;
                            }
                        }
                    }
                },
                new Response.ErrorListener() {
                    @Override
                    public void onErrorResponse(VolleyError error) {
                        Log.d(TAG, "Message could not be sent: " + error.toString());
                    }
                });
        queue.add(request);
    }

    // Get JSON data from server
    public void getRequestUserCoords() {
        String url = "http://" + ip_address + ":3000/usercoords";

        Log.d(TAG, "Begin requesting JSON data");

        JsonArrayRequest request = new JsonArrayRequest(url,
                new Response.Listener<JSONArray>() {
                    @Override
                    public void onResponse(JSONArray response) {
                        Log.d(TAG, "Response is: " + response.toString());
                        try {
                            other_lat = ((JSONObject) (response.get(0))).getDouble(LAT);
                            other_lon = ((JSONObject) response.get(0)).getDouble(LON);
                        } catch (JSONException e) {
                            e.printStackTrace();
                        }
                    }
                },
                new Response.ErrorListener() {
                    @Override
                    public void onErrorResponse(VolleyError error) {
                        Log.d(TAG, "Message could not be sent: " + error.toString());
                    }
                });
        queue.add(request);
    }



}
