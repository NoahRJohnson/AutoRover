package edu.ncf.AutoRover;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacv.CanvasFrame;
import org.bytedeco.javacv.FrameGrabber;
import org.bytedeco.javacv.OpenCVFrameConverter;

import static org.bytedeco.javacpp.opencv_core.IPL_DEPTH_8U;

/**
 * Created by Noah on 2/8/2017.
 */
public class Video implements Runnable {

    private static final Logger LOG = LogManager.getLogger(Video.class);

    public static void main(String[] args) {
        Video v = new Video();
        v.run();
    }
    public void run() {

        try {
            // The available FrameGrabber classes include OpenCVFrameGrabber (opencv_videoio),
            // DC1394FrameGrabber, FlyCaptureFrameGrabber, OpenKinectFrameGrabber, OpenKinect2FrameGrabber,
            // RealSenseFrameGrabber, PS3EyeFrameGrabber, VideoInputFrameGrabber, and FFmpegFrameGrabber.
            FrameGrabber grabber = FrameGrabber.createDefault(0);
            grabber.start();

            // CanvasFrame, FrameGrabber, and FrameRecorder use Frame objects to communicate image data.
            // We need a FrameConverter to interface with other APIs (Android, Java 2D, or OpenCV).
            OpenCVFrameConverter.ToIplImage converter = new OpenCVFrameConverter.ToIplImage();

            // FAQ about IplImage and Mat objects from OpenCV:
            // - For custom raw processing of data, createBuffer() returns an NIO direct
            //   buffer wrapped around the memory pointed by imageData, and under Android we can
            //   also use that Buffer with Bitmap.copyPixelsFromBuffer() and copyPixelsToBuffer().
            // - To get a BufferedImage from an IplImage, or vice versa, we can chain calls to
            //   Java2DFrameConverter and OpenCVFrameConverter, one after the other.
            // - Java2DFrameConverter also has static copy() methods that we can use to transfer
            //   data more directly between BufferedImage and IplImage or Mat via Frame objects.
            opencv_core.IplImage grabbedImage = converter.convert(grabber.grab());
            int width  = grabbedImage.width();
            int height = grabbedImage.height();
            opencv_core.IplImage grayImage    = opencv_core.IplImage.create(width, height, IPL_DEPTH_8U, 1);
            opencv_core.IplImage rotatedImage = grabbedImage.clone();

            // CanvasFrame is a JFrame containing a Canvas component, which is hardware accelerated.
            // It can also switch into full-screen mode when called with a screenNumber.
            // We should also specify the relative monitor/camera response for proper gamma correction.
            CanvasFrame frame = new CanvasFrame("Some Title", CanvasFrame.getDefaultGamma()/grabber.getGamma());

            while (frame.isVisible() && (grabbedImage = converter.convert(grabber.grab())) != null) {

            }
            frame.dispose();
            grabber.stop();
        } catch (org.bytedeco.javacv.FrameGrabber.Exception e) {
            LOG.error(e);
        }

    }
}
