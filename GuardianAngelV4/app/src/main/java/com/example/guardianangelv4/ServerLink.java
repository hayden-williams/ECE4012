package com.example.guardianangelv4;

import android.app.Activity;
import android.util.Log;

import com.android.volley.DefaultRetryPolicy;
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


}
