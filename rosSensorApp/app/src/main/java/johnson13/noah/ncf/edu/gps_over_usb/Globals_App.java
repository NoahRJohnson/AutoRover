package johnson13.noah.ncf.edu.gps_over_usb;

import android.app.Application;

import java.io.PrintWriter;
import java.util.Scanner;

/**
 * Created by Noah on 11/4/2016.
 */

public class Globals_App extends Application {

    private Scanner socketIn = null;
    private PrintWriter socketOut = null;
    private Boolean connected = false;

    public Scanner getSocketIn() {
        return socketIn;
    }
    public PrintWriter getSocketOut() {
        return socketOut;
    }
    public Boolean getConnected() {
        return connected;
    }

    public void setSocketIn(Scanner sIn) {
        this.socketIn = sIn;
    }
    public void setSocketOut(PrintWriter sOut) {
        this.socketOut = sOut;
    }
    public void setConnected(Boolean c) {
        this.connected = c;
    }

    public void closeSockets() {
        if (socketIn != null)
            socketIn.close();
        if (socketOut != null)
            socketOut.close();
        setConnected(false);
    }
}
