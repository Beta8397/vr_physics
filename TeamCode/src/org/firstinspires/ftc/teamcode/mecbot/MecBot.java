package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.AngleUtils;

/**
 * The MechBot class represents a mecanum-wheeled robot with a BNO055IMU and a color sensor. It has methods
 * that provide the basic functionality of such a robot
 */
public class MecBot {

    /*
     * Constants
     */

    private final MotorType MOTOR_TYPE;
    private final float GEAR_RATIO;
    private final float TAN_ALPHA;
    private final BNO055Enhanced.AxesMap AXES_MAP;
    private final BNO055Enhanced.AxesSign AXES_SIGN;
    private final float WHEEL_CIRCUMFERENCE;
    private final float TICKS_PER_ROTATION;
    private final float WL_AVG;

    public static final float MAX_TICKS_PER_SECOND = 2500;

    /*
     * Drive Motors
     */
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    /*
     * The BNO055Enhanced (gyro)
     */
    BNO055Enhanced imu;

    /*
     * The heading offset (this gives us flexibility in specifying the world coordinate system).
     */
    private float headingOffsetRadians = 0;

    /*
     * The current pose of the robot
     */
    private Pose pose = new Pose(0, 0, 0);

    /*
     * The most recent previous readings of the drive motor ticks
     */
    int ticksBL, ticksFL, ticksFR, ticksBR;

    /**
     * Enum MotorType represents types of motors that can be used to power the drive.
     */
    public enum MotorType {
        Neverest40(1120, false),
        Neverest20(560, false),
        NeverestOrbital20(560, true),
        Neverest60(1680, false);

        MotorType(double ticksPerRotation, boolean reversed){
            this.ticksPerRotation = ticksPerRotation;
            this.reversed = reversed;
        }

        private double ticksPerRotation;
        private boolean reversed;
    }

    /**
     * Constructor: USE THIS ONE FOR A REAL ROBOT!
     * @param mType     Motor Type
     * @param w         Wheel Base Width (inches)
     * @param l         Wheel Base Length (inches)
     * @param wheelDiam     Wheel Diameter (inches)
     * @param rollerAngle   Wheel Roller Angle (degrees)
     * @param gearRatio     Gear Ratio (motor output shaft rotations per wheel rotation)
     * @param axesMap       Axes Map for the BNO055Enhanced
     * @param axesSign      Axes Sign for the BNO055Enhanced
     */
    public MecBot(MotorType mType, float w, float l, float wheelDiam, float rollerAngle, float gearRatio, BNO055Enhanced.AxesMap axesMap, BNO055Enhanced.AxesSign axesSign){
        MOTOR_TYPE = mType;
        TAN_ALPHA = (float)Math.tan(Math.toRadians(rollerAngle));
        WL_AVG = 0.5f*w + 0.5f*l/TAN_ALPHA;
        WHEEL_CIRCUMFERENCE = (float)Math.PI * wheelDiam;
        GEAR_RATIO = gearRatio;
        AXES_MAP = axesMap;
        AXES_SIGN = axesSign;
        TICKS_PER_ROTATION = (float)mType.ticksPerRotation;
    }

    /**
     *  No-argument constructor:  USE THIS ONE FOR THE SIMULATOR!!
     */
    public MecBot(){
        MOTOR_TYPE = MotorType.NeverestOrbital20;
        TAN_ALPHA = 1;
        WL_AVG = 15;
        WHEEL_CIRCUMFERENCE = 4*(float)Math.PI;
        GEAR_RATIO = 1;
        AXES_MAP = BNO055Enhanced.AxesMap.XYZ;
        AXES_SIGN = BNO055Enhanced.AxesSign.PPP;
        TICKS_PER_ROTATION = (float)MOTOR_TYPE.ticksPerRotation;
    }

    /**
     * Obtain instances of the robot hardware using the hardware map, and initialize the BNO055IMU
     * @param hwMap
     */
    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backRight = hwMap.get(DcMotor.class, "back_right_motor");

        /*
         * Either the right or the left motors need to have their directions reversed, depending upon
         * the MotorType
         */
        if(MOTOR_TYPE.reversed){
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        /*
         * Set Mode of all four drive motors to RUN_USING_ENCODER. That way, calls to setPower result in PIDF
         * control to keep the motor at the specified speed (as a fraction of Max), rather than raw power.
         */
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(BNO055Enhanced.class, "imu");

        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.axesMap = AXES_MAP;
        parameters.axesSign = AXES_SIGN;

        imu.initialize(parameters);
    }

    /**
     * Return current robot position and orientation as a Pose object
     * @return robot pose
     */
    public Pose getPose() {
        return pose;
    }

    /*
     * Methods that provide access to the individual motors, mainly needed for diagnostics
     */
    public DcMotor getBackLeft() {return backLeft;}
    public DcMotor getFrontLeft() {return frontLeft;}
    public DcMotor getFrontRight() {return frontRight;}
    public DcMotor getBackRight() {return backRight;}

    /**
     * Obtain the current robot heading using the IMU (note use of the heading offset)
     * @return heading in radians
     */
    public float getHeadingRadians(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return headingOffsetRadians + orientation.firstAngle;
    }

    /**
     * Set the robot's heading to the specified value, in degrees. Store this heading in the Pose object, adjust the
     * headingOffsetRadians value as necessary, and update the tick readings of the drive motors.
     * @param headingDegrees
     */
    public void setHeadingDegrees(float headingDegrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        headingOffsetRadians = (float)Math.toRadians(headingDegrees) - orientation.firstAngle;
        pose = new Pose(pose.x, pose.y, (float)Math.toRadians(headingDegrees));
        updateTicks();
    }

    public void setPose(float x, float y, float headingDegrees){
        pose = new Pose(x, y, pose.theta);
        setHeadingDegrees(headingDegrees);
    }

    public void setPose(float x, float y){
        pose = new Pose(x, y, pose.theta);
        updateTicks();
    }

    /**
     * Update the tick readings of the drive motors.
     */
    public void updateTicks(){
        ticksBL = backLeft.getCurrentPosition();
        ticksFL = frontLeft.getCurrentPosition();
        ticksFR = frontRight.getCurrentPosition();
        ticksBR = backRight.getCurrentPosition();
    }

    /**
     * Set the drive powers for the forward, rotation, and strafe directions. Note that these powers are in
     * the range of -1 to +1, and represent the FRACTION of maximal possible drive speed in each direction.
     * @param px    rightward strafe (robot-X-axis) power
     * @param py    forward power (Y-direction)
     * @param pa    rotation power (counter-clockwise if positive)
     */
    public void setDrivePower(float px, float py, float pa) {
        float frontLeftPower = py - pa + px;
        float frontRightPower = py + pa - px;
        float backLeftPower = py - pa - px;
        float backRightPower = py + pa + px;

        float largest = 1;
        largest = Math.max(largest, Math.abs(frontLeftPower));
        largest = Math.max(largest, Math.abs(frontRightPower));
        largest = Math.max(largest, Math.abs(backLeftPower));
        largest = Math.max(largest, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / largest);
        frontRight.setPower(frontRightPower / largest);
        backLeft.setPower(backLeftPower / largest);
        backRight.setPower(backRightPower / largest);
    }

    /**
     * Set robot drive speeds in INCHES PER SECOND (for vx and vy) or RADIANS PER SECOND (for va)
     * @param vx    Drive speed along robot-X-axis (rightward if positive)
     * @param vy    Drive speed along robot-Y-axis (forward if positive)
     * @param va    Rotation speed in radians/sec (counter-clockwise if positive)
     */
    public void setDriveSpeed(float vx, float vy, float va){
        float px = (vx / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION *GEAR_RATIO / (MAX_TICKS_PER_SECOND * TAN_ALPHA);
        float py = (vy / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION * GEAR_RATIO / MAX_TICKS_PER_SECOND;
        float pa = (WL_AVG * va / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION * GEAR_RATIO / MAX_TICKS_PER_SECOND;
        setDrivePower(px, py, pa);
    }

    /**
     * Update robot pose (position on the field, and heading) for a single iteration of a control loop. This is a
     * gyro-assisted odometry algorithm.
     * @return
     */
    public Pose updateOdometry(){
        /*
         * Obtain new heading using the IMU. Then calculate the (small) interval change in the heading, as well as
         * the average heading during the (small) interval of time since the previous iteration. Note the use of the
         * AngleUtils.normalizeRadians method to keep angles in the -PI to +PI range.
         */
        float heading = getHeadingRadians();
        float headingChange = (float)AngleUtils.normalizeRadians(heading - pose.theta);
        float avgHeading = (float)AngleUtils.normalizeRadians(pose.theta + 0.5 * headingChange);

        /*
         * Get the current drive motor ticks
         */
        int currBL = backLeft.getCurrentPosition();
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        /*
         * Calculate the NEW ticks that have occurred since the previous update (new = current - previous)
         */
        int newBL = currBL - ticksBL;
        int newFL = currFL - ticksFL;
        int newFR = currFR - ticksFR;
        int newBR = currBR - ticksBR;

        /*
         * Update the fields that store the tick values, so they will be ready for the next iteration.
         */
        ticksBL = currBL;
        ticksFL = currFL;
        ticksFR = currFR;
        ticksBR = currBR;

        /*
         * Determine the distance that the surface of each wheel has rolled.
         *
         * NOTE: Analysis of DIMENSIONS can be helpful in getting this right. We want to get from "new ticks" to
         * "inches of travel".
         *
         * newBL has dimensions of ticks, and TICKS_PER_ROTATION has dimensions of ticks/rotation.
         * So (newBL / TICKS_PER_ROTATION) has dimensions of Rotations.
         * WHEEL_CIRCUMFERENCE has dimensions of inches/rotation.
         * So (newBL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE has dimensions of  inches.
         *
         */
        float sBL = (newBL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        float sFL = (newFL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        float sFR = (newFR / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        float sBR = (newBR / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE / GEAR_RATIO;

        /*
         * Determine small increment of robot motion in ROBOT COORDINATE SYSTEM
         */
        float dXR = 0.25f * (-sBL + sFL - sFR + sBR) * TAN_ALPHA;
        float dYR = 0.25f * (sBL + sFL + sFR + sBR) * TAN_ALPHA;

        /*
         * Convert this small increment of robot motion into WORLD COORDINATES
         */
        float dX = dXR * (float)Math.sin(avgHeading) + dYR * (float)Math.cos(avgHeading);
        float dY = -dXR * (float)Math.cos(avgHeading) + dYR * (float)Math.sin(avgHeading);

        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        pose = new Pose(pose.x + dX, pose.y + dY, heading);

        /*
         * Return the updated Pose object
         */
        return pose;
    }
}
