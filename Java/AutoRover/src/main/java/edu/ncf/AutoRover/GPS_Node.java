package edu.ncf.AutoRover;

/**
 * Created by Noah on 2/7/2017.
 */
public class GPS_Node {

    private double lat, lng;

    public GPS_Node(double latitude, double longitude) {
        this.lat = latitude;
        this.lng = longitude;
    }

    public double getLat() {
        return this.lat;
    }

    public double getLng() {
        return this.lng;
    }


}
