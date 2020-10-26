package org.firstinspires.ftc.teamcode.cv;

/**
 * Created by JimLori on 11/6/2016.
 */


//import com.qualcomm.ftcrobotcontroller.R;
//import com.vuforia.CameraDevice;
//import com.vuforia.Image;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.lang.annotation.Target;
import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;


/**
 * VuforiaNavigator class contains static methods for navigating using Vuforia. It also contains methods
 * for obtaining images from the video stream.
 *
 * Now updated to allow use of either the built-in phone camera or an external webcam.
 *
 */
public class VuforiaNavigator {

//    private static VuforiaLocalizer vuforia;
//    private static VuforiaTrackables targets;
//    private static String targetAssetName = null;
//    private static final OpenGLMatrix DEFAULT_TARGET_LOCATION = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
//            AxesOrder.XYX, AngleUnit.DEGREES, 0, 0, 0));
//    public static final OpenGLMatrix DEFAULT_PHONE_LOCATION_ON_ROBOT =
//            OpenGLMatrix.translation(0, 0, 0).multiplied(
//                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                            AngleUnit.DEGREES, 90, 0, 0));
//    private static OpenGLMatrix phoneLocationOnRobot = DEFAULT_PHONE_LOCATION_ON_ROBOT;
//    private static final VuforiaLocalizer.CameraDirection DEFAULT_CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
//
//    /**
//     * Create an instance of VuforiaLocalizer, load the Trackables (using default location for each Trackable).
//     * For each Trackable, set its phone location and camera direction to default values.
//     *
//     * @param assetName  Asset name for trackables (null if using only to obtain images)
//     * @param webcamName Webcam to use (null for built-in phone camera)
//     */
//    public static void activate(String assetName, WebcamName webcamName) {
//        internalActivate(assetName, null, null, DEFAULT_CAMERA_DIRECTION, webcamName);
//    }
//
//
//    /**
//     * Create an instance of VuforiaLocalizer, load the Trackables (setting the location of each Trackable).
//     * For each Trackable, set its phone information. Finally, activate the Trackables.
//     *
//     * @param assetName Asset name for trackables
//     * @param targetLocations Array of target locations relative to origin of field coordinate system
//     * @param phoneLocation Phone location relative to origin of robot coordinate system
//     * @param cameraDirection Phone camera direction (no effect if using external webcam
//     * @param webcamName Webcam to use (null for built-in phone camera)
//     */
//    public static void activate(String assetName, OpenGLMatrix[] targetLocations, OpenGLMatrix phoneLocation,
//                                VuforiaLocalizer.CameraDirection cameraDirection, WebcamName webcamName) {
//        internalActivate(assetName, targetLocations, phoneLocation, cameraDirection, webcamName);
//    }
//
//
//    /**
//     * Create an instance of VuforiaLocalizer, load the Trackables (setting the location of each Trackable).
//     * For each Trackable, set its phone information. Finally, activate the Trackables.
//     */
//    private static void internalActivate(String assetName, OpenGLMatrix[] targetLocations, OpenGLMatrix phoneLocation,
//                                         VuforiaLocalizer.CameraDirection cameraDirection, WebcamName webcamName) {
//        targetAssetName = assetName;
//        if (phoneLocation != null) VuforiaNavigator.phoneLocationOnRobot = phoneLocation;
//
//        //Create the VuforiaLocalizer
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        parameters.useExtendedTracking = false;
//        if(webcamName == null) {
//            parameters.cameraDirection = cameraDirection;
//        } else {
//            parameters.cameraName = webcamName;
//        }
//        //New key that supports webcams
//        parameters.vuforiaLicenseKey = "YOUR LICENSE KEY IS REQUIRED HERE!!!";
//
//
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        //Set up the VuforiaLocalizer to allow frame grabs of RGB565 images
//        vuforia.setFrameQueueCapacity(3);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//
//        //Get targets, set all target locations, and set phone info for each target
//        if (assetName == null || assetName.isEmpty()) {
//            //If there is no given asset name just return.
//            return;
//        }
//        targets = vuforia.loadTrackablesFromAsset(targetAssetName);
//        for (int i = 0; i < targets.size(); i++) {
//            if (targetLocations == null) targets.get(i).setLocation(DEFAULT_TARGET_LOCATION);
//            else targets.get(i).setLocation(targetLocations[i]);
//            ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).setPhoneInformation(phoneLocationOnRobot, cameraDirection);
//        }
//
//        //activate targets
//        targets.activate();
//
//    }
//
//    public static OpenGLMatrix getTargetPoseRelativeToCamera(int i){
//        OpenGLMatrix targetPoseRelativeToCamera = ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).getPose();
//        return targetPoseRelativeToCamera;
//    }
//
//
//
//    /**
//     * Obtain pose (as OpenGLMatrix) of the specified target relative to the robot.
//     *
//     * @param i Index of the target.
//     * @return Pose of target relative to robot (i.e., the target-to-robot coordinate transform)
//     */
//    public static OpenGLMatrix getTargetPoseRelativeToRobot(int i) {
//        OpenGLMatrix targetPoseRelativeToCamera =
//                ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).getPose();
//        if (targetPoseRelativeToCamera == null) return null;
//        return phoneLocationOnRobot.multiplied(targetPoseRelativeToCamera);
//    }
//
//    /**
//     * Obtain pose of (as OpenGLMatrix) of robot relative to specified target.
//     *
//     * @param i Index of the target.
//     * @return Pose of robot relative to target (i.e., the robot-to-target coordinate transform)
//     */
//    public static OpenGLMatrix getRobotPoseRelativeToTarget(int i) {
//        OpenGLMatrix targetPoseRelativeToCamera =
//                ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).getPose();
//        if (targetPoseRelativeToCamera == null) return null;
//        return (phoneLocationOnRobot.multiplied(targetPoseRelativeToCamera)).inverted();
//    }
//
//    /**
//     * Set target location on field.
//     *
//     * @param i              Index of the target.
//     * @param targetLocation Target location as OpenGLMatrix (i.e., the target-to-field coordinate transform)
//     */
//    public static void setTargetLocation(int i, OpenGLMatrix targetLocation) {
//        targets.get(i).setLocation(targetLocation);
//    }
//
//    /**
//     * Get robot location on field, using the specified target
//     *
//     * @param i Index of the target
//     * @return Pose of robot on field (i.e., the robot-to-field coordinate transform)
//     */
//    public static OpenGLMatrix getRobotLocation(int i) {
//        return ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).getRobotLocation();
//    }
//
//
//    /**
//     * Provided with a robot-to-target coordinate transform (i.e., a robot pose relative to target),
//     * returns the Z,X coordinates and heading Phi of the robot in the target coordinate system.
//     * Here, Phi is the angle between target Z-axis and projection of robot Y-axis into the target
//     * Z-X plane.
//     *
//     * @param locationTransform robot-to-target coordinate transform (i.e., robot pose relative to target)
//     * @return Array of float containing Z, X, and Phi (radians), in that order.
//     */
//    public static float[] getZ_X_Phi_FromLocationTransform(OpenGLMatrix locationTransform) {
//        float[] locationData = locationTransform.getData();
//        float[] returnValue = new float[3];
//        returnValue[0] = locationData[14] / 10.0f;
//        returnValue[1] = locationData[12] / 10.0f;
//        returnValue[2] = (float) Math.atan2(locationData[4], locationData[6]);
//        return returnValue;
//    }
//
//    /**
//     * Provided with a robot-to-field coordinate transform (i.e., a robot pose relative to field),
//     * returns the X,Y coordinates and heading Theta of the robot in the field coordinate system.
//     * Here, Theta is the angle between the field X axis and the projection of the robot Y axis into
//     * the field X-Y plane.
//     *
//     * @param locationTransform robot-to-field coordinate transform (i.e., robot pose relative to field)
//     * @return Array of float containing X,Y, and Theta (radians), in that order
//     */
//    public static float[] getX_Y_Theta_FromLocationTransform(OpenGLMatrix locationTransform) {
//        float[] locationData = locationTransform.getData();
//        float[] returnValue = new float[3];
//        returnValue[0] = locationData[12] / 10.0f;
//        returnValue[1] = locationData[13] / 10.0f;
//        returnValue[2] = (float) Math.atan2(locationData[5], locationData[4]);
//        return returnValue;
//    }
//

    /**
     * Take any angle, in radians, and normalize it to the -pi to +pi range.
     *
     * @param angle Angle to be normalized.
     * @return Normalized value.
     */
    public static double NormalizeAngle(double angle) {
        double temp = (angle + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }

    public static double NormalizeDegrees(double angle) {
        double temp = (angle + 180.0) / 360.0;
        return (temp - Math.floor(temp) - 0.5) * 360.0;

    }

    public static double normalizeDegrees360(double angle){
        double temp = angle / 360.0;
        return (temp - Math.floor(temp)) * 360.0;
    }
//
//
//    /**
//     * returns the frame queue of the VuforiaLocalizer object.
//     */
//    public static BlockingQueue<VuforiaLocalizer.CloseableFrame> getFrameQueue() {
//        return vuforia.getFrameQueue();
//    }
//
//    /**
//     * Clears the frame queue.
//     *
//     * @param frameQueue
//     */
//    public static void clearFrameQueue
//    (BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
//        VuforiaLocalizer.CloseableFrame tempFrame = null;
//        while (true) {
//            tempFrame = frameQueue.poll();
//            if (tempFrame == null) break;
//            tempFrame.close();
//        }
//    }
//
//    /**
//     * Given the frame queue, and desired image width and height, clears all but the most recent frame
//     * in the queue, then searches that frame for an RGB565 image of specified width and height. If
//     * such an image is found, loads the destination array with the image pixel values.
//     *
//     * @param frameQueue The frame queue.
//     * @param width      Desired image width.
//     * @param height     Desired image height.
//     * @param dst        Byte array to hold the RGB565 image pixels, two bytes per pixel, little-endian format.
//     * @return true if image was obtained, otherwise false.
//     */
//    public static boolean getRGB565Array
//    (BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int width, int height,
//     byte[] dst) {
//        if (dst.length != (2 * width * height)) return false;
//        VuforiaLocalizer.CloseableFrame frame = null;
//        VuforiaLocalizer.CloseableFrame tempFrame = null;
//        Image img = null;
//        try {
//            //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
//            while (true) {
//                tempFrame = frameQueue.poll();
//                if (tempFrame == null) break;
//                if (frame != null) frame.close();
//                frame = tempFrame;
//            }
//            if (frame == null) return false;
//
//            //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
//            long numImages = frame.getNumImages();
//            for (int i = 0; i < numImages; i++) {
//                img = frame.getImage(i);
//                if (img.getFormat() == PIXEL_FORMAT.RGB565 && img.getWidth() == width && img.getHeight() == height) {
//                    ByteBuffer byteBuf = img.getPixels();
//                    byteBuf.get(dst);
//                    return true;
//                }
//            }
//            return false;
//        } finally {
//            if (frame != null) frame.close();
//            if (tempFrame != null) tempFrame.close();
//        }
//    }
//
//    /**
//     * Turn flashlight on or off
//     *
//     * @param on true to turn light on, false to turn off.
//     * @return true if flashlight was turned on or off as requested, otherwise false.
//     */
//    public static boolean setFlashTorchMode(boolean on) {
//        return CameraDevice.getInstance().setFlashTorchMode(on);
//
//    }
//
//    /**
//     * Determine whether specified target is currently visible.
//     *
//     * @param i Index of target.
//     * @return True if target visible (or if position can be inferred with extended tracking), false if not.
//     */
//    public static boolean isTargetVisible(int i) {
//        return ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).isVisible();
//    }
//
//
//
//    public static int[] getRGBfromByteArray(int row, int col, int width, byte[] src){
//        int index = 2 * (row * width + col);
//        byte b1 = src[index];
//        byte b2 = src[index+1];
//        int blue = (b1 & 0x1F) << 3;
//        int red = (b2 & 0xF8);
//        int green = ((b1 & 0xE0) >> 3) + ((b2 & 0x7) << 5);
//        return new int[] {red, green, blue};
//    }
//
//
//    public static Bitmap getBitmap(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue)
//            throws InterruptedException{
//        VuforiaLocalizer.CloseableFrame frame = null;
//        try{
//            frame = frameQueue.poll(10, TimeUnit.MICROSECONDS);
//            if (frame == null) return null;
//            long numImages = frame.getNumImages();
//            Image rgbImage = null;
//            for (int i = 0; i < numImages; i++)
//                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    rgbImage = frame.getImage(i);
//                    break;
//                }
//            if (rgbImage == null) return null;
//            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(rgbImage.getPixels());
//            return bm;
//        }
//        catch(InterruptedException exc){
//            throw exc;
//        }
//        finally{
//            if (frame != null) frame.close();
//        }
//    }
//

}
