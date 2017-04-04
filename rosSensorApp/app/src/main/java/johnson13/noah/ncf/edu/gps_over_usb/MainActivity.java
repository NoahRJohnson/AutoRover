package johnson13.noah.ncf.edu.gps_over_usb;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
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

/**
 * Main Activity, launched on app startup
 */
public class MainActivity extends RosActivity implements View.OnClickListener {


    private static final String MASTER_URI = "http://localhost/38300";
    public static final String TAG = "MainActivity";
    public static final int TIMEOUT = 10;
    Intent i = null;
    TextView tv = null;
    private String connectionStatus = null;
    private Handler mHandler = null;
    ServerSocket server = null;

    public MainActivity() {
        super("Publish GPS and IMU sensor data via ROS", "RosSensors");
        //URI.create(MASTER_URI) to hardcode master uri
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connection);

        //Set up click listeners for the buttons
        View connectButton = findViewById(R.id.connect_button);
        connectButton.setOnClickListener(this);

        i = new Intent(this, Connected.class);
        mHandler = new Handler();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // A unique part for ROS
        TalkerNode talkerNode = new TalkerNode(this);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
                InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(talkerNode, nodeConfiguration);
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.connect_button:
                if (!isLocationEnabled()) {
                    showLocationAlert();
                    break;
                } else { // if Location is enabled
                    // setup socket connection in a new separate thread, which will change activities
                    Runnable r = new initializeConnection((Globals_App) this.getApplication());
                    new Thread(r).start();
                    String msg = "Attempting to connect…";
                    Toast.makeText(MainActivity.this, msg, Toast.LENGTH_LONG).show();
                    break;
                }
        }
    }

    private class initializeConnection implements Runnable {

        private Globals_App global_app_ref;

        public initializeConnection(Globals_App gar) {
            this.global_app_ref = gar;
        }

        public void run() {
            Socket client=null;
            // initialize server socket
            try{
                server = new ServerSocket(38300);
                server.setSoTimeout(TIMEOUT*1000);

                //attempt to accept a connection
                client = server.accept();
                global_app_ref.setSocketIn(new Scanner(client.getInputStream()));
                global_app_ref.setSocketOut(new PrintWriter(client.getOutputStream(), true));
            } catch (SocketTimeoutException e) {
                // print out TIMEOUT
                connectionStatus = "Connection has timed out! Please try again";
                mHandler.post(showConnectionStatus);
            } catch (IOException e) {
                Log.e(TAG, ""+e);
            } finally {
                //close the server socket
                try {
                    if (server!=null)
                        server.close();
                } catch (IOException ec) {
                    Log.e(TAG, "Cannot close server socket"+ec);
                }
            }

            if (client!=null) {
                global_app_ref.setConnected(true);
                // print out success
                connectionStatus = "Connection was successful!";
                mHandler.post(showConnectionStatus);

                startActivity(i);
            }
        }

    }

    private boolean isLocationEnabled() {
        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
    }

    private void showLocationAlert() {
        final AlertDialog.Builder dialog = new AlertDialog.Builder(this);
        dialog.setTitle("Enable Location")
                .setMessage("Your Locations Settings is set to 'Off'.\nPlease Enable Location to " +
                        "use this app")
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
    }

    /**
     * Pops up a “toast” to indicate the connection status
     */
    private Runnable showConnectionStatus = new Runnable() {
        public void run() {
            Toast.makeText(MainActivity.this, connectionStatus, Toast.LENGTH_SHORT).show();
        }
    };
}

