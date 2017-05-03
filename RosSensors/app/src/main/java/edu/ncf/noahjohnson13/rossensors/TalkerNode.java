package edu.ncf.noahjohnson13.rossensors;

import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.widget.TextView;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;

import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;



import org.ros.concurrent.CancellableLoop;
import org.ros.node.topic.Publisher;

import java.security.Security;

import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static android.Manifest.permission.ACCESS_FINE_LOCATION;

/**
 * Created by noah on 3/20/17.
 *
 * Once MasterChooser a connection between this android 'server' and the
 * processing unit housing the ROS master has been created,
 * this node publishes consistent IMU and GPS data to the
 * ??? topic, which the robot_localization package listens
 * to.
 */
public class TalkerNode extends AbstractNodeMain implements
        SensorEventListener,
        GoogleApiClient.ConnectionCallbacks,
        GoogleApiClient.OnConnectionFailedListener,
        LocationListener {

    /**
     * What topic to publish IMU messages to.
     */
    private final String IMU_TOPIC_NAME = "imu/data";

    /**
     * What topic to publish NavSatFix messages to.
     */
    private final String GPS_TOPIC_NAME = "gps/fix";
    /**
     * What frame id to put in the header of messages.
     * This specifies the frame with respect to which
     * the data is being reported. Since all messages
     * being published from this app are physically
     * readings sensors at the phone's location, we
     * will use a frame defined in the system as /phone.
     */
    private final String FRAME_ID = "phone";
    /**
     * The ROS name for this node that the ROS master will
     * advertise.
     */
    private final String NODE_NAME = "PhoneNode";
    /**
     * Sensors allow a sampling period to be set, which specifies
     * the suggested delay between sensor events (i.e., delay
     * between onSensorChanged() calls).
     * SENSOR_DELAY_NORMAL      200 ms delay = 5 Hz
     * SENSOR_DELAY_UI          60 ms delay = 16 Hz
     * SENSOR_DELAY_GAME        20 ms delay = 50 Hz
     * SENSOR_DELAY_FASTEST     0 ms artificial delay between events
      */
    private final int IMU_SENSORS_SAMPLING_PERIOD = SensorManager.SENSOR_DELAY_NORMAL;
    /**
     * Specifies how long events can be stuck in the hardware FIFO queue,
     * before being reported. This shouldn't need to be modified.
      */
    private final int IMU_SENSORS_MAX_REPORT_LATENCY = SensorManager.SENSOR_DELAY_UI;
    /**
     * Hardcode value for Nexus 4, from paper
     */
    private final double GYROSCOPE_VARIANCE = 0.004;
    /**
     * Hardcode value for Nexus 4, from paper
     */
    private final double ACCELEROMETER_VARIANCE = 0.04;
    /**
     * .01 deg = 0.00017 rad, totally random guess
     */
    private final double ORIENTATION_VARIANCE = 0.00017;
    /**
     * In milliseconds
     */
    private final int DESIRED_GPS_INTERVAL = 5 * 1000;
    /**
     *
     */
    private final int FASTEST_GPS_INTERVAL = 2 * 1000;
    /**
     *
     */
    private GoogleApiClient mGoogleApiClient;
    /**
     *
     */
    private LocationRequest mLocationRequest;

    /**
     *
     */
    private SensorManager mSensorManager;

    /**
     *
     */
    private final float[] mAccelerometerReading = new float[4];
    /**
     *
     */
    private final float[] mGyroscopeReading = new float[3];
    /**
     *
     */
    private final float[] mGravityValues = new float[3];

    /**
     *
     */
    private final float[] mMagnetometerReading = new float[3];




    /**
     *
     */
    private boolean accelerometerUsed;
    /**
     *
     */
    private boolean magnetometerUsed;
    /**
     *
     */
    private boolean gyroscopeUsed;
    /**
     *
     */
    private boolean gravityUsed;
    /**
     *
     */
    private boolean orientationQuaternionUsed;


    /**
     *
     */
    private boolean accelerometerUpdated = false;
    /**
     *
     */
    private boolean magnetometerUpdated = false;
    /**
     *
     */
    private boolean gyroscopeUpdated = false;
    /**
     *
     */
    private boolean gravityUpdated = false;
    /**
     *
     */
    private boolean orientationQuaternionUpdated = false;

    /**
     *
     */
    private final double[] accelerometerCovariance = new double[9];
    /**
     *
     */
    private final double[] gyroscopeCovariance = new double[9];
    /**
     * Combines magnetometer, accelerometer, and gyroscopic readings.
     */
    private final double[] orientationQuaternionCovariance = new double[9];

    /**
     *
     */
    private final float[] mOrientationQuaternion = new float[4];
    /**
     *
     */
    private Log ROS_LOG;
    /**
     *
     */
    private ConnectedNode connectedNode;
    /**
     *
     */
    private Context context;
    /**
     *
     */
    private Publisher<sensor_msgs.Imu> imu_publisher;
    /**
     *
     */
    private Publisher<sensor_msgs.NavSatFix> gps_publisher;



    /**
     *
     * @param context
     */
    public TalkerNode(Context context) {
        this.context = context;
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

    }


    /**
     * For RosCore
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }


    /**
     * Called once the node has started and has
     * successfully connected to the ROS master.
     * @param connectedNode
     */
    @Override
    public void onStart(final ConnectedNode connectedNode) {

        this.connectedNode = connectedNode;
        ROS_LOG = connectedNode.getLog();

        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        /*
         * Get updates from the accelerometer and magnetometer at a constant rate.
         * To make batch operations more efficient and reduce power consumption,
         * provide support for delaying updates to the application via the
         * MAX_REPORT_LATENCY argument.
         *
         * In this example, the sensor reporting delay is small enough such that
         * the application receives an update before the system checks the sensor
         * readings again.
         */
        Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor gravitySensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        Sensor orientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        if (accelerometer == null) {
            accelerometerUsed = false;
            accelerometerUpdated = true; // leave true so we don't check it
            ROS_LOG.error("Accelerometer sensor unavailable.");
        } else {
            accelerometerUsed = true;
            accelerometerUpdated = false;
            mSensorManager.registerListener(this, accelerometer,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (magnetometer == null) {
            magnetometerUsed = false;
            magnetometerUpdated  = true; // leave true so we don't check it
            ROS_LOG.error("Magnetometer sensor unavailable.");
        } else {
            magnetometerUsed = true;
            magnetometerUpdated  = false;
            mSensorManager.registerListener(this, magnetometer,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (gyroscope == null) {
            gyroscopeUsed = false;
            gyroscopeUpdated = true; // leave true so we don't check it
            ROS_LOG.error("Gyroscope sensor unavailable.");
        } else {
            gyroscopeUsed = true;
            gyroscopeUpdated = false;
            mSensorManager.registerListener(this, gyroscope,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (gravitySensor == null) {
            gravityUsed = false;
            gravityUpdated = true; // leave true so we don't check it
            ROS_LOG.error("Gyroscope sensor unavailable.");
        } else {
            gravityUsed = true;
            gravityUpdated = false;
            mSensorManager.registerListener(this, gravitySensor,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (orientation == null) {
            orientationQuaternionUsed = false;
            orientationQuaternionUpdated = true; // leave true so we don't check it
            ROS_LOG.error("Rotation vector sensor unavailable.");
        } else {
            orientationQuaternionUsed = true;
            orientationQuaternionUpdated = false;
            mSensorManager.registerListener(this, orientation,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }

        // Create publishers
        imu_publisher = connectedNode.newPublisher(IMU_TOPIC_NAME, sensor_msgs.Imu._TYPE);
        gps_publisher = connectedNode.newPublisher(GPS_TOPIC_NAME, sensor_msgs.NavSatFix._TYPE);


        // Now handle the setup for GPS callbacks

        createLocationRequest(); // initialize mLocationRequest field

        // Create an instance of GoogleAPIClient.
        if (mGoogleApiClient == null) {
            mGoogleApiClient = new GoogleApiClient.Builder(context)
                    .addConnectionCallbacks(this)
                    .addOnConnectionFailedListener(this)
                    .addApi(LocationServices.API)
                    .build();
        }
        mGoogleApiClient.connect();
        // Once googleAPI's onConnected() callback is called,
        // location requests will be started, and onLocationChanged()
        // will publish NavSatFix messages.

        // Normally would do a cancellable loop here. But we want this thread
        // to be open for receiving asynchronous event handler callbacks.
    }


    /**
     * Called for unrecoverable error.
     * @param node
     * @param throwable
     */
    @Override
    public void onError(Node node, Throwable throwable) {}


    /**
     * Called when this node has started to shut down.
     * @param node
     */
    @Override
    public void onShutdown(Node node) {
        // Don't receive any more updates from either IMU sensor.
        if (mSensorManager != null) {
            mSensorManager.unregisterListener(this);
        }

        // Don't receive any more updates from GPS.
        if (mGoogleApiClient != null) {
            stopLocationUpdates();
            mGoogleApiClient.disconnect();
        }

        // De-register the publisher with ROS MASTER
        imu_publisher.shutdown();
        gps_publisher.shutdown();
    }


    /**
     * Called once this node has finished shutting down.
     * @param node
     */
    @Override
    public void onShutdownComplete(Node node) {}


    /**
     * When IMU accuracy changes
     * @param sensor
     * @param accuracy
     */
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        //TODO affect IMU covariance by this
        switch(accuracy) {

            case SensorManager.SENSOR_STATUS_UNRELIABLE:
                break;

            case SensorManager.SENSOR_STATUS_NO_CONTACT:
                break;

            case SensorManager.SENSOR_STATUS_ACCURACY_LOW:
                break;

            case SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM:
                break;

            case SensorManager.SENSOR_STATUS_ACCURACY_HIGH:
                break;


        }
    }


    /**
     * Handles new readings from this device's accelerometer and magnetometer.
     * To simplify calculations, consider storing these readings as unit vectors.
     *
     * From docs:
     * "Called when there is a new sensor event. Note that 'on changed'
     * is somewhat of a misnomer, as this will also be called if we have a new
     * reading from a sensor with the exact same sensor values (but a
     * newer timestamp)."
     *
     * Thus even if the sensor readings haven't changed, this handler will still
     * get called as often as we specified when we registered with the sensor
     * manager.
     *
     * Every time a new accelerometer and magnetometer reading have both occurred,
     * calculate the phone's orientation and publish an IMU message.
     *
     * @param event
     */
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event == null || event.sensor == null) {
            ROS_LOG.error("Null sensor event.");
            return;
        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, mAccelerometerReading,
                    0, event.values.length);
            accelerometerUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, mMagnetometerReading,
                    0, event.values.length);
            magnetometerUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            System.arraycopy(event.values, 0, mGravityValues,
                    0, event.values.length);
            gravityUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            System.arraycopy(event.values, 0, mGyroscopeReading,
                    0, event.values.length);
            gyroscopeUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // Get a normalized quaternion representing orientation in ENU frame
            System.arraycopy(event.values, 0, mOrientationQuaternion,
                    0, mOrientationQuaternion.length); // [x,y,z,w]
            orientationQuaternionUpdated = true;
        }

        if (accelerometerUpdated && magnetometerUpdated &&
                gyroscopeUpdated && gravityUpdated &&
                orientationQuaternionUpdated) {
            /*
             * Orientation (calculated using accelerometer and magnetometer)
             * linear acceleration (accelerometer) and
             * angular velocity (gyroscope) readings compose
             * a ROS IMU message. Since unused sensors have
              * their Updated variables set to true, then reaching
              * this code block implies that each sensor being used
              * has updated once since the last publish, so publish
              * a new IMU message now with the latest readings.
             */
            sensor_msgs.Imu imu_msg = imu_publisher.newMessage();
            fill_IMU_msg(imu_msg);
            imu_publisher.publish(imu_msg);

            // If a sensor is used, reset its updated flag to false.
            // If a sensor is not being used, then leave the updated
            // flag as true so that we don't wait for it to update.
            accelerometerUpdated = !accelerometerUsed;
            magnetometerUpdated = !magnetometerUsed;
            gyroscopeUpdated = !gyroscopeUsed;
            gravityUpdated = !gravityUsed;
            orientationQuaternionUpdated = !orientationQuaternionUsed;
        }
    }


    /**
     * http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
     * http://docs.ros.org/api/std_msgs/html/msg/Header.html
     * http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
     * http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
     * @param msg Fills in its fields in place. Assumed to have
     *                  sub-messages created.
     */
    private void fill_IMU_msg(sensor_msgs.Imu msg) {

        /* Fill in the header. The sequence is automatically
         generated as an increasing int. */
        msg.getHeader().setStamp(connectedNode.getCurrentTime());
        msg.getHeader().setFrameId(FRAME_ID);

        // Already in ENU frame
        msg.getOrientation().setX(mOrientationQuaternion[0]);
        msg.getOrientation().setY(mOrientationQuaternion[1]);
        msg.getOrientation().setZ(mOrientationQuaternion[2]);
        msg.getOrientation().setW(mOrientationQuaternion[3]);

        // Modify accelerometer data to follow REP-103, and be in ENU frame.
        // Accelerometer values are given in phone's local frame.
        // They include gravity.
        // Change the device relative acceleration values to earth relative values
        // X axis -> East
        // Y axis -> North Pole
        // Z axis -> Sky
        float[] enuAcc = new float[16];
        if (accelerometerUsed && gravityUsed && magnetometerUsed) {
            float[] R = new float[16], I = new float[16];

            SensorManager.getRotationMatrix(R, I, mGravityValues, mMagnetometerReading);

            float[] inv = new float[16];

            mAccelerometerReading[3] = 0;

            android.opengl.Matrix.invertM(inv, 0, R, 0);
            android.opengl.Matrix.multiplyMV(enuAcc, 0, inv, 0, mAccelerometerReading, 0);
        }
        else { // will set cov[0] to -1, and tell ekf node to ignore this value
            enuAcc[0] = 0;
            enuAcc[1] = 0;
            enuAcc[2] = 0;
        }

        // Set linear acceleration in ENU frame
        msg.getLinearAcceleration().setX(enuAcc[0]);
        msg.getLinearAcceleration().setY(enuAcc[1]);
        msg.getLinearAcceleration().setZ(enuAcc[2]);

        // Gyroscope values are given in phone's local frame, increasing
        // CCW (good). The phone is laying face up on top of the rover, so
        // its frame's z axis is the same as in the ENU frame, and so yaw'
        // from the gyroscope is the same. roll' and pitch' are ignored by
        // the state estimation node since rover is 2D, therefore we don't
        // bother converting gyroscope readings from phone frame to ENU frame.

        if (!gyroscopeUsed) { // will set cov[0] to -1, and tell ekf node to ignore this value
            mGyroscopeReading[0] = 0;
            mGyroscopeReading[1] = 0;
            mGyroscopeReading[2] = 0;
        }

        msg.getAngularVelocity().setX(mGyroscopeReading[0]);
        msg.getAngularVelocity().setY(mGyroscopeReading[1]);
        msg.getAngularVelocity().setZ(mGyroscopeReading[2]);


        // initialize covariance matrices to all zeros
        for (int i=0; i < 9; i++) {
            orientationQuaternionCovariance[i] = 0;
            gyroscopeCovariance[i] = 0;
            accelerometerCovariance[i] = 0;
        }

        // Per message specs, if sensor isn't used, put a -1 in
        // the first element of the flattened covariance matrix
        if (orientationQuaternionUsed) { // in ENU frame
            orientationQuaternionCovariance[0] = ORIENTATION_VARIANCE * ORIENTATION_VARIANCE;
            orientationQuaternionCovariance[4] = ORIENTATION_VARIANCE * ORIENTATION_VARIANCE;
            orientationQuaternionCovariance[8] = ORIENTATION_VARIANCE * ORIENTATION_VARIANCE;
        }
        else
            orientationQuaternionCovariance[0] = -1;
        if (gyroscopeUsed) { // in phone frame, z axis is all we care about and it's the same as ENU frame.
            gyroscopeCovariance[0] = GYROSCOPE_VARIANCE * GYROSCOPE_VARIANCE;
            gyroscopeCovariance[4] = GYROSCOPE_VARIANCE * GYROSCOPE_VARIANCE;
            gyroscopeCovariance[8] = GYROSCOPE_VARIANCE * GYROSCOPE_VARIANCE;
        }
        else
            gyroscopeCovariance[0] = -1;
        if (accelerometerUsed && gravityUsed && magnetometerUsed) { // phone frame has z the same, so x,y->E,N is just simple 1-1 mapping, and we can use the same variances
            accelerometerCovariance[0] = ACCELEROMETER_VARIANCE * ACCELEROMETER_VARIANCE;
            accelerometerCovariance[4] = ACCELEROMETER_VARIANCE * ACCELEROMETER_VARIANCE;
            accelerometerCovariance[8] = ACCELEROMETER_VARIANCE * ACCELEROMETER_VARIANCE;
        }
        else
            accelerometerCovariance[0] = -1;

        msg.setOrientationCovariance(orientationQuaternionCovariance);
        msg.setAngularVelocityCovariance(gyroscopeCovariance);
        msg.setLinearAccelerationCovariance(accelerometerCovariance);
    }

    private void fill_NavSatFix_message(sensor_msgs.NavSatFix msg,
                                        Location fix) {

        assert(fix != null);

        msg.getHeader().setStamp(connectedNode.getCurrentTime());
        msg.getHeader().setFrameId(FRAME_ID);

        msg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
        msg.getStatus().setService(NavSatStatus.SERVICE_GPS);

        /*if (fix.hasBearing())
            bearing = fix.getBearing(); // Get the bearing, in degrees.
        if (fix.hasSpeed())
            speed = fix.getSpeed(); // Get the speed if it is available, in meters/second over ground.
        */
        // lat and long in degrees, guaranteed to be
        // available since fix != null
        msg.setLatitude(fix.getLatitude());
        msg.setLongitude(fix.getLongitude());

        if (fix.hasAltitude()) // altitude in meters
            msg.setAltitude(fix.getAltitude()); // Get the altitude if available, in meters above the WGS 84 reference ellipsoid.
        else
            msg.setAltitude(0); // rover will be on ground, so default to 0

	// initialize covariance matrix to all zeros
        double[] fix_covariance = new double[9];
        for (int i = 0; i < fix_covariance.length; i++)
            fix_covariance[i] = 0;

        if (fix.hasAccuracy()) {
            // We'll approximate the covariance as a diagonal matrix
            // with the reported horizontal accuracy (in meters) squared
            // for both lat and long, to get m^2 variance as needed
            msg.setPositionCovarianceType(NavSatFix.COVARIANCE_TYPE_APPROXIMATED);

            float horiz_accuracy = fix.getAccuracy(); // Get the estimated accuracy of this location, in meters.
            /*if (fix.hasVerticalAccuracy()) {
                    vert_accuracy = fix.getVerticalAccuracy(); // Get the estimated accuracy of this location, in meters.
               } else { }
             */
            fix_covariance[0] = horiz_accuracy * horiz_accuracy;
            fix_covariance[4] = horiz_accuracy * horiz_accuracy;
            //fix_covariance[8] = vert_accuracy * vert_accuracy;
            // TODO: Find a way to get vertical accuracy and report it, before ekf_localization node turns on 3D
        }
        else { // no accuracy estimation, so we can't create covariance matrix. So leave it as all 0s and flag it to be ignored.
            msg.setPositionCovarianceType(NavSatFix.COVARIANCE_TYPE_UNKNOWN);
        }
        msg.setPositionCovariance(fix_covariance);

    }

    /**
     * Specify parameters for our request for periodic GPS location updates
     */
    protected void createLocationRequest() {
        mLocationRequest = new LocationRequest();
        mLocationRequest.setInterval(DESIRED_GPS_INTERVAL);
        mLocationRequest.setFastestInterval(FASTEST_GPS_INTERVAL);
        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
    }

    /**
     * Turn off LocationListener callbacks.
     */
    private void stopLocationUpdates() {
        LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, this);
    }

    @Override
    /**
     * Callback for LocationListener.
     * Called when the location is updated.
     */
    public void onLocationChanged(Location location) {

        if (location != null) {
            sensor_msgs.NavSatFix gps_msg = gps_publisher.newMessage();
            fill_NavSatFix_message(gps_msg, location);
            gps_publisher.publish(gps_msg);
        }
        /*
        // update the last time updated with the current time
        mLastUpdateTime = DateFormat.getTimeInstance().format(new Date());
        */
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
        /*try {
            mCurrentLocation = LocationServices.FusedLocationApi.getLastLocation(
                    mGoogleApiClient);
        } catch (SecurityException e) {
            ROS_LOG.error(e.getMessage());
        }*/

        startLocationUpdates();
    }


    /**
     * Make a request for location updates. From here on LocationListener
     * callbacks should be invoked.
     */
    protected void startLocationUpdates() {
        try {
            LocationServices.FusedLocationApi.requestLocationUpdates(
                    mGoogleApiClient, mLocationRequest, this);
        } catch (SecurityException e) {
            ROS_LOG.error(String.format("Security exception when starting location updates: %s", e.getMessage()));
        }
    }

    @Override
    /**
     * Called when GoogleAPIClient is suspended.
     */
    public void onConnectionSuspended(int cause) {
        ROS_LOG.warn("Google API Client suspended");
    }

    @Override
    /**
     * Called when GoogleAPIClient cannot connect to the
     * Google Play Service.
     */
    public void onConnectionFailed(ConnectionResult result) {
        ROS_LOG.warn("Google API Client suspended");
    }






}
