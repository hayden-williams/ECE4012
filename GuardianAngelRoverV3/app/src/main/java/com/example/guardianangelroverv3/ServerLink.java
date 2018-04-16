package com.example.guardianangelroverv3;

import android.util.Log;

import com.android.volley.NetworkResponse;
import com.android.volley.ParseError;
import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.HttpHeaderParser;
import com.android.volley.toolbox.JsonArrayRequest;
import com.android.volley.toolbox.JsonObjectRequest;
import com.android.volley.toolbox.Volley;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.UnsupportedEncodingException;
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
    public static final String LENGTH = "distance";
    public static final String BEARING = "bearing";

    public static final String MESSAGE_TYPE_ROVER = "0";
    public static final String MESSAGE_TYPE_USER = "1";
    public static final String MESSAGE_TYPE_END_TRIP = "2";
    public static final String MESSAGE_TYPE_EMERGENCY = "3";
    public static final String MESSAGE_TYPE_ROVER_ARRIVED = "4";
    public static final String MESSAGE_TYPE_WAYFINDING = "5";
    public static final String MESSAGE_TYPE_PATH_FINISHED = "6";

    public static final String DIR[] = {"direction1", "direction2", "direction3", "direction4", "direction5"};
    public static final String LEN[] = {"distance1", "distance2", "distance3", "distance4", "distance5"};


    private String ip_address = "128.61.7.199";
    private String TAG = "ServerLink";
    private RequestQueue queue;

    public double other_lat = 0;
    public double other_lon = 0;
    public double rover_lat = 0;
    public double rover_lon = 0;


    public String user_name = "USER";

    public boolean emergency = false;
    public boolean arrived = false;
    public boolean ended = false;
    public boolean gotHome = false;
    public boolean requested = false;

    protected ServerLink() {
        queue = Volley.newRequestQueue(CurrentActivity.getInstance().getCurrentActivity());
    }

    public void setIPAddress(String ip) {
        ip_address = ip;
    }

    // Send JSON data to server
    public void request(List<StringPair> jsonList) {
        String url = "http://" + ip_address + ":3000/guardian-angel";

        Log.d(TAG, "Begin sending JSON data");

        JSONObject jsonBody = new JSONObject();
        try {
            for (StringPair listItem : jsonList) {
                jsonBody.put(listItem.getName(), listItem.getValue());
            }
            Log.d(TAG, "Sending JSON: " + jsonBody.toString());
        } catch (JSONException e) {
            Log.d(TAG, "JSONException: " + e.toString());
        }

        JsonObjectRequest request = new JsonObjectRequest(Request.Method.POST, url, jsonBody,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        if (response == null) {
                            Log.d(TAG, "Message sent, no response.");
                        } else {
                            Log.d(TAG, "Response is: " + response.toString());
                        }
                    }
                },
                new Response.ErrorListener() {
                    @Override
                    public void onErrorResponse(VolleyError error) {
                        Log.d(TAG, "Message could not be sent: " + error.toString());
                    }
                }) {
            @Override
            protected Response<JSONObject> parseNetworkResponse(NetworkResponse response) {
                try {
                    String jsonString = new String(response.data,
                            HttpHeaderParser.parseCharset(response.headers, PROTOCOL_CHARSET));

                    JSONObject result = null;

                    if (jsonString != null && jsonString.length() > 0)
                        result = new JSONObject(jsonString);

                    return Response.success(result,
                            HttpHeaderParser.parseCacheHeaders(response));
                } catch (UnsupportedEncodingException e) {
                    return Response.error(new ParseError(e));
                } catch (JSONException je) {
                    return Response.error(new ParseError(je));
                }
            }
        };
        queue.add(request);
    }



    // Get JSON data from server
    public void getRequestUserCoords() {
        String url = "http://" + ip_address + ":3000/usercoord";

        Log.d(TAG, "Begin requesting JSON data");

        JsonObjectRequest request = new JsonObjectRequest(url, null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        if (response == null) {
                            Log.d(TAG, "Message sent, no response.");
                        } else {
                            Log.d(TAG, "Response is: " + response.toString());
                            try {
                                other_lat = response.getDouble(LAT);
                                other_lon = response.getDouble(LON);
                                Log.d(TAG, "Successfully parsed response, lat is " + other_lat + " and lon is " + other_lon);
                            } catch (JSONException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }, new Response.ErrorListener() {
            @Override
            public void onErrorResponse(VolleyError error) {
                Log.d(TAG, "Message could not be sent: " + error.toString());
                error.printStackTrace();
            }
        });

        queue.add(request);
    }

    // Get JSON data from server
    public void getRequestState() {
        String url = "http://" + ip_address + ":3000/rover";

        Log.d(TAG, "Begin requesting JSON data");

        JsonObjectRequest request = new JsonObjectRequest(url, null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        if (response == null) {
                            Log.d(TAG, "Message sent, no response.");
                        } else {
                            Log.d(TAG, "Response is: " + response.toString());
                            try {
                                emergency = response.getInt("emergency") == 1;
                                arrived = response.getInt("arrived") == 1;
                                ended = response.getInt("ended") == 1;
                                requested = response.getInt("goToUser") == 1;
                                //gotHome = response.getInt("gotHome") == 1;
                                Log.d(TAG, "Successfully parsed response, lat is " + other_lat + " and lon is " + other_lon);
                            } catch (JSONException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }, new Response.ErrorListener() {
            @Override
            public void onErrorResponse(VolleyError error) {
                Log.d(TAG, "Message could not be sent: " + error.toString());
                error.printStackTrace();
            }
        });

        queue.add(request);
    }

    // Get JSON data from server
    public void getRequestRoverCoords() {
        String url = "http://" + ip_address + ":3000/rovercoord";

        Log.d(TAG, "Begin requesting JSON data");

        JsonObjectRequest request = new JsonObjectRequest(url, null,
                new Response.Listener<JSONObject>() {
                    @Override
                    public void onResponse(JSONObject response) {
                        if (response == null) {
                            Log.d(TAG, "Message sent, no response.");
                        } else {
                            Log.d(TAG, "Response is: " + response.toString());
                            try {
                                rover_lat = response.getDouble(LAT);
                                rover_lon = response.getDouble(LON);
                                Log.d(TAG, "Successfully parsed response, lat is " + other_lat + " and lon is " + other_lon);
                            } catch (JSONException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }, new Response.ErrorListener() {
            @Override
            public void onErrorResponse(VolleyError error) {
                Log.d(TAG, "Message could not be sent: " + error.toString());
                error.printStackTrace();
            }
        });

        queue.add(request);
    }



}
