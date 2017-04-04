package johnson13.noah.ncf.edu.gps_over_usb;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;

import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;

import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;



import org.ros.concurrent.CancellableLoop;
import org.ros.node.topic.Publisher;
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
{

    /**
     * What topic to publish IMU messages to.
     */
    private final String IMU_TOPIC_NAME = "chat";
    /**
     * What frame id to put in the header of IMU messages.
     */
    private final String IMU_FRAME_ID = "imu";
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
     *
     */
    private GoogleApiClient mGoogleApiClient;
    /**
     *
     */
    private Object location_changed_lock;
    /**
     *
     */
    private boolean shutdown = false;
    /**
     *
     */
    private Location mCurrentLocation;
    /**
     *
     */
    private LocationRequest mLocationRequest;
    /**
     *
     */
    private static final int MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 2;

    /**
     *
     */
    private SensorManager mSensorManager;
    /**
     *
     */
    private final float[] mAccelerometerReading = new float[3];
    /**
     *
     */
    private final float[] mMagnetometerReading = new float[3];
    /**
     *
     */
    private final float[] mGyroscopeReading = new float[3];



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
    private final float[] mRotationMatrix = new float[9];
    /**
     *
     */
    private final float[] mOrientationQuaternion = new float[4];
    /**
     *
     */
    private Log LOG;
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
        LOG = connectedNode.getLog();

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
        System.out.println("LETS REGISTER!!!");
        Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor orientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        if (accelerometer == null) {
            accelerometerUsed = false;
            accelerometerUpdated = true; // leave true so we don't check it
            LOG.error("Accelerometer sensor unavailable.");
        } else {
            accelerometerUsed = true;
            mSensorManager.registerListener(this, accelerometer,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (magnetometer == null) {
            magnetometerUsed = false;
            magnetometerUpdated  = true; // leave true so we don't check it
            LOG.error("Magnetometer sensor unavailable.");
        } else {
            magnetometerUsed = true;
            mSensorManager.registerListener(this, magnetometer,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (gyroscope == null) {
            gyroscopeUsed = false;
            gyroscopeUpdated = true; // leave true so we don't check it
            LOG.error("Gyroscope sensor unavailable.");
        } else {
            gyroscopeUsed = true;
            mSensorManager.registerListener(this, gyroscope,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }
        if (orientation == null) {
            orientationQuaternionUsed = false;
            orientationQuaternionUpdated = true; // leave true so we don't check it
            LOG.error("Rotation vector sensor unavailable.");
        } else {
            orientationQuaternionUsed = true;
            mSensorManager.registerListener(this, orientation,
                    IMU_SENSORS_SAMPLING_PERIOD, IMU_SENSORS_MAX_REPORT_LATENCY);
        }

        imu_publisher = connectedNode.newPublisher(IMU_TOPIC_NAME, sensor_msgs.Imu._TYPE);
        //System.out.println("JUST MADE PUBLISHER");
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
        // Don't receive any more updates from either sensor.
        mSensorManager.unregisterListener(this);

        // De-register the publisher with ROS MASTER
        imu_publisher.shutdown();
    }


    /**
     * Called once this node has finished shutting down.
     * @param node
     */
    @Override
    public void onShutdownComplete(Node node) {}


    /**
     *
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
            LOG.error("Null sensor event.");
            return;
        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, mAccelerometerReading,
                    0, mAccelerometerReading.length);
            accelerometerUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, mMagnetometerReading,
                    0, mMagnetometerReading.length);
            magnetometerUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            System.arraycopy(event.values, 0, mGyroscopeReading,
                    0, mGyroscopeReading.length);
            gyroscopeUpdated = true;
        } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // Translate rotation vector to a normalized quaternion, stored as [w,x,y,z]
            mSensorManager.getQuaternionFromVector(mOrientationQuaternion, event.values);
            orientationQuaternionUpdated = true;
        }

        if (accelerometerUpdated && magnetometerUpdated &&
                gyroscopeUpdated && orientationQuaternionUpdated) {
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

            // If a sensor is used, set its updated flag to false.
            // If a sensor is not being used, then leave the updated
            // flag as true so that we don't wait for it to update.
            accelerometerUpdated = !accelerometerUsed;
            magnetometerUpdated = !magnetometerUsed;
            gyroscopeUpdated = !gyroscopeUsed;
            orientationQuaternionUpdated = !orientationQuaternionUsed;
        }
    }


    /*
    public void updateOrientation() {
        // Update rotation matrix, which is needed to update orientation angles.
        mSensorManager.getRotationMatrix(mRotationMatrix, null,
                mAccelerometerReading, mMagnetometerReading);

        // Calculate orientation angles (azimuth, pitch, roll)
        float[] orientationAngles = new float[3];
        mSensorManager.getOrientation(mRotationMatrix, orientationAngles);


    }*/


    /**
     * http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
     * http://docs.ros.org/api/std_msgs/html/msg/Header.html
     * http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
     * http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
     * @param msg Fills in its fields in place. Assumed to have
     *                  sub-messages created.
     */
    private void fill_IMU_msg(sensor_msgs.Imu msg) {

/*        System.out.println(String.format(
                "Seq, timestamp, frameid: %d, %s, %s",
                msg.getHeader().getSeq(),
                msg.getHeader().getStamp().toString(),
                msg.getHeader().getFrameId()));
*/
        /* Fill in the header. The sequence is automatically
         generated as an increasing int.
         TODO: check that I'm using timestamp correctly, check ros wiki on tf */
        //msg.getHeader().setSeq(imu_seq);
        msg.getHeader().setStamp(connectedNode.getCurrentTime());
        msg.getHeader().setFrameId("imu");

        msg.getOrientation().setW(mOrientationQuaternion[0]);
        msg.getOrientation().setX(mOrientationQuaternion[1]);
        msg.getOrientation().setY(mOrientationQuaternion[2]);
        msg.getOrientation().setZ(mOrientationQuaternion[3]);

        msg.getAngularVelocity().setX(mGyroscopeReading[0]);
        msg.getAngularVelocity().setY(mGyroscopeReading[1]);
        msg.getAngularVelocity().setZ(mGyroscopeReading[2]);

        msg.getLinearAcceleration().setX(mAccelerometerReading[0]);
        msg.getLinearAcceleration().setY(mAccelerometerReading[1]);
        msg.getLinearAcceleration().setZ(mAccelerometerReading[2]);


        // TODO: remove this, get actual covariance measurements
        for (int i=0; i < 9; i++) {
            orientationQuaternionCovariance[i] = 0;
            gyroscopeCovariance[i] = 0;
            accelerometerCovariance[i] = 0;
        }

        // Per message specs, if sensor isn't used, put a -1 in
        // the first element of the flattened covariance matrix
        if (!orientationQuaternionUsed)
            orientationQuaternionCovariance[0] = -1;
        if (!gyroscopeUsed)
            gyroscopeCovariance[0] = -1;
        if (!accelerometerUsed)
            accelerometerCovariance[0] = -1;

        msg.setOrientationCovariance(orientationQuaternionCovariance);
        msg.setAngularVelocityCovariance(gyroscopeCovariance);
        msg.setLinearAccelerationCovariance(accelerometerCovariance);
    }

    private void publish_GPS_message() {
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
            /*out.write(String.format("%f:%f:%f:%f:%f:%f",
                    accuracy, altitude, bearing, speed,
                    lat, lng));*/
        }
    }





}
