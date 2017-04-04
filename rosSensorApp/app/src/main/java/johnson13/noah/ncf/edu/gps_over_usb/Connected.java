package johnson13.noah.ncf.edu.gps_over_usb;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.location.Location;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.View;
import android.widget.TextView;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.PendingResult;
import com.google.android.gms.common.api.ResultCallback;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.location.LocationSettingsRequest;
import com.google.android.gms.location.LocationSettingsResult;

import java.io.PrintWriter;

import static android.Manifest.permission.ACCESS_FINE_LOCATION;

/**
 * Now that the usb tcp port connection has been made, collect gps data and send it over the socket
 * Created by Noah on 11/4/2016.
 */

public class Connected extends Activity implements
        GoogleApiClient.ConnectionCallbacks,
        GoogleApiClient.OnConnectionFailedListener,
        LocationListener {

    private GoogleApiClient mGoogleApiClient;
    private Object location_changed_lock;
    private boolean shutdown = false;
    private Location mCurrentLocation;
    private LocationRequest mLocationRequest;
    private static final int MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 2;


    private class writeToSocketThread implements Runnable {
        private PrintWriter out;


        public writeToSocketThread(Globals_App gar) {
            this.out = gar.getSocketOut();
        }

        public void run() {
            while (!shutdown) {
                try {
                    // wait until the activity's main thread signals that a new location
                    // has been found.
                    location_changed_lock.wait();
                } catch (java.lang.InterruptedException e) {
                    ; // do nothing,
                }
                write_out_current_location();
            }
        }

        private void write_out_current_location() {
            String data_over_socket = "";
            float accuracy = -1; // in meters
            double altitude = -1; // in meters
            float bearing = -1; // in degrees
            float speed = -1; // in meters/second

            // lat and lng are guaranteed to be there, as
            // long as the location is not null
            double lat;
            double lng;

            if (mCurrentLocation == null) {
                ; // TODO: Some error logging?
            } else {
                // we are in a thread so synchronize to stop dirty reads
                synchronized (mCurrentLocation) {
                    // lat and long in degrees
                    if (mCurrentLocation.hasAccuracy())
                        accuracy = mCurrentLocation.getAccuracy(); // Get the estimated accuracy of this location, in meters.
                    if (mCurrentLocation.hasAltitude())
                        altitude = mCurrentLocation.getAltitude(); // Get the altitude if available, in meters above the WGS 84 reference ellipsoid.
                    if (mCurrentLocation.hasBearing())
                        bearing = mCurrentLocation.getBearing(); // Get the bearing, in degrees.
                    if (mCurrentLocation.hasSpeed())
                        speed = mCurrentLocation.getSpeed(); // Get the speed if it is available, in meters/second over ground.

                    lat = mCurrentLocation.getLatitude();
                    lng = mCurrentLocation.getLongitude();
                }
                out.write(String.format("%f:%f:%f:%f:%f:%f",
                        accuracy, altitude, bearing, speed,
                        lat, lng));
            }
        }

    }

    @Override
    /**
     * Called when Activity is created, before it is displayed
     */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connected); // use activity_connected XML file

        // Create an instance of GoogleAPIClient.
        if (mGoogleApiClient == null) {
            mGoogleApiClient = new GoogleApiClient.Builder(this)
                    .addConnectionCallbacks(this)
                    .addOnConnectionFailedListener(this)
                    .addApi(LocationServices.API)
                    .build();
        }

        createLocationRequest(); // initialize mLocationRequest field

        Globals_App globals = (Globals_App) this.getApplication();
        if (globals.getConnected()) {
            System.out.println("CONNECTED");

            // Start async thread to write out GPS data over the socket connection
            Runnable r = new writeToSocketThread(globals);
            new Thread(r).start();

            System.out.println("Thread started.");
        } else {
            System.err.println("NOT CONNECTED");
            disconnect(); // exit
        }
    }

    /**
     * Specify parameters for our request for periodic GPS location updates
     */
    protected void createLocationRequest() {
        mLocationRequest = new LocationRequest();
        mLocationRequest.setInterval(10000);
        mLocationRequest.setFastestInterval(5000);
        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
    }

    /**
     * Make a request for location updates. From here on LocationListener
     * callbacks should be invoked.
     */
    protected void startLocationUpdates() {
        if (ContextCompat.checkSelfPermission(this,
                ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {

            // Should we show an explanation?
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                    ACCESS_FINE_LOCATION)) {

                // Show an expanation to the user *asynchronously* -- don't block
                // this thread waiting for the user's response! After the user
                // sees the explanation, try again to request the permission.

            } else {

                // No explanation needed, we can request the permission.

                ActivityCompat.requestPermissions(this,
                        new String[]{ACCESS_FINE_LOCATION},
                        MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION);

                // MY_PERMISSIONS_REQUEST_READ_CONTACTS is an
                // app-defined int constant. The callback method gets the
                // result of the request.
            }
        }
        LocationServices.FusedLocationApi.requestLocationUpdates(
                mGoogleApiClient, mLocationRequest, this);
    }

    /**
     * Turn off LocationListener callbacks.
     */
    protected void stopLocationUpdates() {
        LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, this);
    }

    @Override
    /**
     * Callback for LocationListener.
     * Called when the location is updated.
     */
    public void onLocationChanged(Location location) {

        // update the mCurrentLocation field
        synchronized (mCurrentLocation) {
            mCurrentLocation = location;
        }

        //update the UI in a new thread
        updateUILocation();

        // signal the waiting socket thread to wake up and write the new location out to the socket
        location_changed_lock.notify();

        /*
        // update the last time updated with the current time
        mLastUpdateTime = DateFormat.getTimeInstance().format(new Date());
        */
    }

    private void updateUILocation() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                TextView lonText = (TextView) findViewById(R.id.longitudeValueGPS);
                TextView latText = (TextView) findViewById(R.id.latitudeValueGPS);

                lonText.setText(String.valueOf(mCurrentLocation.getLongitude()));
                latText.setText(String.valueOf(mCurrentLocation.getLatitude()));
            }
        });
    }

    @Override
    /**
     * Connection callback method.
     * Called when Google API Client is ready.
     * Check permissions for GPS access.
     * Then call the getLastLocation() function to set
     * initial location. Then call startLocationUpdates()
     * to get periodic location callbacks.
     */
    public void onConnected(Bundle connectionHint) {
        if (ContextCompat.checkSelfPermission(this,
                ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {

            System.err.print("No location permissions!");
            disconnect();
        }

        mCurrentLocation = LocationServices.FusedLocationApi.getLastLocation(
                mGoogleApiClient);
        if (mCurrentLocation != null) {
            updateUILocation();
        }

        startLocationUpdates();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // permission was granted, yay! Do the
                    // location-related task you need to do.

                } else {

                    // permission denied, boo! Disable the
                    // functionality that depends on this permission.
                }
                return;
            }

            // other 'case' lines to check for other
            // permissions this app might request
        }
    }

    @Override
    public void onConnectionSuspended(int cause) {
        disconnect();
    }

    @Override
    /**
     * Called when GoogleAPIClient cannot connect to the
     * Google Play Service.
     */
    public void onConnectionFailed(ConnectionResult result) {
        disconnect();
    }

    /**
     * Called by Activity after onCreate, when activity
     * is now being displayed to the user.
     */
    protected void onStart() {
        mGoogleApiClient.connect();
        super.onStart();
    }

    /**
     * Called by Activity after onCreate, when activity
     * is no longer being displayed to the user.
     */
    protected void onStop() {
        mGoogleApiClient.disconnect();
        shutdown = true; // signal socket thread to shutdown
        super.onStop();
    }

    /**
     * Custom disconnect method, called in code and also as a callback
     * when the disconnect button is pressed. Requests the api to stop
     * sending location requests, signals the socket thread to shutdown,
     * deletes this activity, and starts the Connection activity.
     */
    protected void disconnect() {
        if (mGoogleApiClient != null) {
            stopLocationUpdates();
            mGoogleApiClient.disconnect();
        }
        shutdown = true;
        Intent i = new Intent(this, MainActivity.class);
        finish();  //Kill the activity from which you will go to next activity
        startActivity(i);
    }
}

