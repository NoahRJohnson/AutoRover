package edu.ncf.AutoRover;

import org.apache.log4j.Logger;

import com.fazecast.jSerialComm.*;

import java.util.Arrays;
import java.util.Scanner;

import static com.fazecast.jSerialComm.SerialPort.NO_PARITY;
import static com.fazecast.jSerialComm.SerialPort.ONE_STOP_BIT;
import static com.fazecast.jSerialComm.SerialPort.TIMEOUT_READ_BLOCKING;


public class Arduino {

    private final static Logger LOG = Logger.getLogger(Arduino.class);

    private double uS_cm_conversion;
    private SerialPort port;

    public Arduino(int tempF) {

        // Calculate the speed of sound in air at this temperature, use it for conversion factor
        double tempC = (tempF - 32) * (5.0/9);
        uS_cm_conversion = 0.03315 + (.00006*tempC); // cm/uS
    }

    public boolean openSerial(int baudRate) {
        SerialPort available_ports[] = SerialPort.getCommPorts();
        int selected_port = userSelectPort(available_ports);;
        while (selected_port == -1) {
            System.out.println("Invalid port selection. Please enter a valid port number.");
            selected_port = userSelectPort(available_ports);
        }
        port = available_ports[selected_port];
        port.setComPortParameters(baudRate, 8, ONE_STOP_BIT, NO_PARITY); // Arduino defaults to 8N1
        port.setComPortTimeouts(TIMEOUT_READ_BLOCKING, 0, 0); // Block indefinitely on readBytes() calls until data is available
        return port.openPort();
    }

    public boolean closeSerial() {
        return port.closePort();
    }

    /**
     *
     * @return
     */
    public int userSelectPort(SerialPort available_ports[]) {
        // determine which serial port to use
        System.out.println("Select a port:");
        int i = 1;
        for(SerialPort port : available_ports) {
            System.out.println(i++ + ". " + port.getSystemPortName());
        }
        Scanner s = new Scanner(System.in);
        int chosenPort = s.nextInt();

        if (chosenPort >= i || chosenPort < 0)
            return -1;
        else
            return chosenPort - 1;
    }

    /**
     * Reads the data pushed from the pushSensorUpdate()
     * Arduino function. The order and number of bytes read
     * here must match those pushed out by the Arduino code.
     */
    public void readSensorUpdates() {
        byte[] command, value;

        try {
            while (true)
            {
                command = new byte[1];
                port.readBytes(command, 1);
                char c = (char) (command[0]  & 0xFF);

                value = new byte[8];
                switch (c) {
                    case 'L': // L == Left motor's quadrature encoder's position value (Arduino long, 4 bytes encoded to 8 hexadecimal ASCII characters)
                        port.readBytes(value, 8);
                        int lmPos = hexASCIIBytesToInt(value, 8);

                        System.out.println(String.format("Left Motor pos: %d", lmPos));
                        break;
                    case 'R': // R == Right motor's quadrature encoder's position value (Arduino long, 4 bytes encoded to 8 hexadecimal ASCII characters)
                        port.readBytes(value, 8);
                        int rmPos =  hexASCIIBytesToInt(value, 8);

                        System.out.println(String.format("Right Motor pos: %d", rmPos));
                        break;
                    case 'S': // S == Angular degree of servo (uint8_t, 1 byte encoded to 2 hexadecimal ASCII characters)
                        port.readBytes(value, 2);
                        int servo_angle = hexASCIIBytesToInt(value, 2); // incoming byte is unsigned, as are java chars. bytes are signed

                        System.out.println(String.format("Servo angle (deg): %d", servo_angle));
                        break;
                    case 'P': // P == Ultrasonic ping time (in micro seconds) measured at the given servo angle (uint16_t, 2 bytes encoded to 4 hexadecimal ASCII characters)
                        port.readBytes(value, 4);
                        int ping_uS = hexASCIIBytesToInt(value, 4);

                        int ping_cm = convertMicroSecondsToCm(ping_uS);
                        System.out.println(String.format("Ping time (ms): %d", ping_cm));
                        break;
                    default:
                        LOG.error(String.format("Invalid command: %c", c));
                }
            }
        } catch (Exception e) { e.printStackTrace(); }
    }


    /**
     * Converts a byte array storing num_nibbles ASCII characters
     * representing a hexadecimal string into a signed Java int (4 bytes).
     * @param b The byte array containing ASCII characters of hex nibbles
     * @param num_nibbles The number of nibbles stored in b
     * @return Int representation of the hex ASCII
     */
    private int hexASCIIBytesToInt(byte[] b, int num_nibbles) {

        assert(num_nibbles <= b.length);

        int ret = 0;

        for (int i=0; i < num_nibbles; i++) {
            if (b[i] >= '0' && b[i] <= '9') {
                ret |= ((b[i] - 48) << ((num_nibbles-i-1)*4));
            } else if (b[i] >= 'a' && b[i] <= 'f') {
                ret |= ((b[i] - 87) << ((num_nibbles-i-1)*4));
            } else {
                LOG.error(String.format("Invalid byte array from Arduino serial: %s", Arrays.toString(b)));
            }
        }
        return ret;
    }

    /**
     * Converts the ping time in microseconds to a distance in centimeters.
     * @return Distance of ping measurement in centimeters
     */
    private int convertMicroSecondsToCm(int ping_uS) {
        return (int) (ping_uS / uS_cm_conversion);
    }

}
