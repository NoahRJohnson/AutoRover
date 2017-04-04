package edu.ncf.AutoRover;

/**
 * Created by Noah on 2/2/2017.
 */
public class Main {

    /**
     * test main
     */
    public static void main(String[] args) {

        Arduino a = new Arduino(80);

        if (a.openSerial(9600)) {
            a.readSensorUpdates();
        }
        a.closeSerial();


    }

    /**
     *
     * @param start
     * @param end
     */
    private void navigateBetweenNodes(GPS_Node start, GPS_Node end) {

    }
}
