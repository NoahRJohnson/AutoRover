/**
 * Created by Noah on 1/29/2017.
 */
import com.fazecast.jSerialComm.*;

import javax.swing.*;
import java.util.Scanner;


public class Arduino {

    private static final String PORT = "COM4";


    /**
     * test main
     */
    public static void main() {

        System.out.println(SerialPort.getCommPorts());
/*
        // create a window with a slider
        JFrame window = new JFrame();
        JSlider slider = new JSlider();
        slider.setMaximum(1023);
        window.add(slider);
        window.pack();
        window.setVisible(true);

        // determine which serial port to use
        SerialPort ports[] = SerialPort.getCommPorts();
        System.out.println("Select a port:");
        int i = 1;
        for(SerialPort port : ports) {
            System.out.println(i++ + ". " + port.getSystemPortName());
        }
        Scanner s = new Scanner(System.in);
        int chosenPort = s.nextInt();

        // open and configure the port
        SerialPort port = ports[chosenPort - 1];
        if(port.openPort()) {
            System.out.println("Successfully opened the port.");
        } else {
            System.out.println("Unable to open the port.");
            return;
        }
        //port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);

        try {
            while (true)
            {
                while (port.bytesAvailable() == 0)
                    Thread.sleep(20);

                byte[] readBuffer = new byte[port.bytesAvailable()];
                int numRead = port.readBytes(readBuffer, readBuffer.length);
                System.out.println("Read " + numRead + " bytes.");
                for (byte b : readBuffer) {
                    System.out.println(b);
                }
            }
        } catch (Exception e) { e.printStackTrace(); }
        port.closePort();
        // enter into an infinite loop that reads from the port and updates the GUI
        Scanner data = new Scanner(port.getInputStream());
        while(data.hasNextLine()) {
            int number = 0;
            try{number = Integer.parseInt(data.nextLine());}catch(Exception e){}
            slider.setValue(number);
        }*/
    }



}
