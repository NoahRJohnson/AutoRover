package edu.ncf.noahjohnson13.rossensors;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URI;
import java.util.Scanner;

import static android.Manifest.permission.ACCESS_FINE_LOCATION;

/**
 * Main Activity, launched on app startup
 */
public class MainActivity extends RosActivity implements View.OnClickListener {


    public static final String TAG = "MainActivity";
    /**
     *
     */
    private static final int MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 2;
    /**
     *
     */
    private TalkerNode talkerNode;


    public MainActivity() {
        super("Publish GPS and IMU sensor data via ROS", "RosSensors");
        //URI.create(MASTER_URI) to hardcode master uri
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //Set up click listeners for the buttons
        View shutdownButton = findViewById(R.id.shutdown_button);
        shutdownButton.setOnClickListener(this);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // A unique part for ROS

        // Check if Location is enabled on the phone. If not, complain
        // and then exit.
        if (ContextCompat.checkSelfPermission(this,
                ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(MainActivity.this,
                    "Location not enabled, please enable.", Toast.LENGTH_SHORT).show();
            shutdownSelf();
        }
        // implicit else

        talkerNode = new TalkerNode(this);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
                InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(talkerNode, nodeConfiguration);
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.shutdown_button:
                shutdownSelf();
                break;
        }
    }

    private void shutdownSelf() {
        android.os.Process.killProcess(android.os.Process.myPid());
        System.exit(1);
    }


    /*private boolean isLocationEnabled() {
        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
    }*/

    /*private void showLocationAlert() {
        final AlertDialog.Builder dialog = new AlertDialog.Builder(this);
        dialog.setTitle("Enable Location")
                .setMessage("Your Locations Settings is set to 'Off'.\nPlease Enable Location to " +
                        "use this app, so that GPS readings can be published.")
                .setPositiveButton("Location Settings", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface paramDialogInterface, int paramInt) {
                        Intent myIntent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                        startActivity(myIntent);
                    }
                })
                .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface paramDialogInterface, int paramInt) {
                    }
                });
        dialog.show();
    }*/
}

