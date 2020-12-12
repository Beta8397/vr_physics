package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.application.Platform;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import odefx.node_with_geom.BoxWithDGeom;
import odefx.node_with_geom.CylWithDGeom;
import odefx.node_with_geom.GroupWithDGeoms;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.ftcfield.UltimateGoalField;
import virtual_robot.util.AngleUtils;
import java.util.*;

import static org.ode4j.ode.OdeConstants.*;

/**
 *  Ultimate Goal game-specific robot configuration
 *
 *  Mecanum-wheel bot with intake and shooter for rings, and grabber for wobbles.
 *
 */
@BotConfig(name = "Ultimate Bot")
public class UltimateBot extends VirtualBot {

    private final float TOTAL_MASS = 15000;  //mg
    private final float TOTAL_Z_INERTIA = 5000000f; //gm*cm2
    private final float FIELD_FRICTION_COEFF = 1.0f;
    private final float GRAVITY = 980f; // cm/s2
    //Max possible force (in Robot-X direction) at any wheel (each wheel gets 1/4 of robot weight)
    private final float MAX_WHEEL_X_FORCE = TOTAL_MASS * GRAVITY * FIELD_FRICTION_COEFF / (4.0f * (float)Math.sqrt(2));

    private final MotorType motorType = MotorType.NeverestOrbital20;
    private DcMotorExImpl[] motors = null;
    private DcMotorExImpl shooterMotor = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
//    private ServoImpl shooterElevServo = null;
    private ServoImpl shooterTrigServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;
    private DcMotorExImpl intakeMotor = null;
    private DcMotorExImpl armMotor = null;
    private ServoImpl handServo = null;

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private GroupWithDGeoms shooter;
    private double shooterElevationAngle = 34;
    private Rotate shooterElevationRotate = new Rotate(shooterElevationAngle, 4, 13, 0, new Point3D(1, 0, 0));
    boolean shooterCocked = true;

    GroupWithDGeoms grabber;
    private double grabberBaseRotation = 180.0;
    private double grabberRotation = grabberBaseRotation;
    private double leftHandRotation = 0;
    private double rightHandRotation = 0;
    private Rotate grabberRotate = new Rotate(grabberRotation, 0, 0, 0, Rotate.X_AXIS);
    private Rotate leftHandRotate = new Rotate(leftHandRotation, 0, 0, 0, Rotate.Z_AXIS);
    private Rotate rightHandRotate = new Rotate(rightHandRotation, 0, 0, 0, Rotate.Z_AXIS);

    List<FxBody> storedRings = new ArrayList<>();

    private FxBody grabbedWobble = null;
    private DHingeJoint wobbleJoint = null;
    double wobbleJointBaseOffset = 0;
    private DAMotorJoint wobbleMotor = null;
    private boolean fingersJustClosed = true;
    private double priorFingerPos = 0;

    private double[][] tWR; //Transform from wheel motion to robot motion

    GeneralMatrixF  M_ForceWheelToRobot;
    MatrixF M_ForceRobotToWheel;

    Rotate[] wheelRotates = new Rotate[] {new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS)};

    double[] wheelRotations = new double[]{0,0,0,0};


    Group botGroup;


    @Override
    public void init(){
        super.init();
        hardwareMap.setActive(true);

        motors = new DcMotorExImpl[]{
                hardwareMap.get(DcMotorExImpl.class, "back_left_motor"),
                hardwareMap.get(DcMotorExImpl.class,"front_left_motor"),
                hardwareMap.get(DcMotorExImpl.class,"front_right_motor"),
                hardwareMap.get(DcMotorExImpl.class,"back_right_motor")
        };

        intakeMotor = hardwareMap.get(DcMotorExImpl.class,"intake_motor");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
//        shooterElevServo = (ServoImpl)hardwareMap.servo.get("shooter_elev_servo");
        shooterTrigServo = (ServoImpl)hardwareMap.servo.get("shooter_trig_servo");
        shooterMotor = hardwareMap.get(DcMotorExImpl.class, "shooter_motor");
        armMotor = hardwareMap.get(DcMotorExImpl.class,"arm_motor");
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");
        handServo.setPosition(1);
        hardwareMap.setActive(false);

        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;

        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/ wlAverage, -0.25/ wlAverage, 0.25/ wlAverage, 0.25/ wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };

        float RRt2 = 0.5f * (float)Math.sqrt(interWheelLength*interWheelLength + interWheelWidth*interWheelWidth) * (float)Math.sqrt(2.0);
        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});

        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

        storeRings();

    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(motorType));
        hardwareMap.put("intake_motor", new DcMotorExImpl(MotorType.Neverest40, false, false));
        hardwareMap.put("shooter_motor", new DcMotorExImpl(MotorType.Neverest40, false, false));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("shooter_elev_servo", new ServoImpl());
        hardwareMap.put("shooter_trig_servo", new ServoImpl());
        hardwareMap.put("arm_motor", new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put("hand_servo", new ServoImpl());
    }

    /**
     * Store rings 4,5,6 "in the robot" (i.e., disable them and add them to the stored rings list for future shooting)
     * This should be called only from the application thread.
     */
    public void storeRings(){
        if (Config.GAME == Config.Game.ULTIMATE_GOAL) {
            List<FxBody> ultimateGoalFieldRings = UltimateGoalField.getInstance().getRings();
            for (int i = 4; i < 7; i++) {
                FxBody ring = ultimateGoalFieldRings.get(i);
                if (storedRings.contains(ring) || storedRings.size() >= 3) continue;
                ring.getGeom("ring").disable();
                ring.disable();
                if (subSceneGroup.getChildren().contains(ring.getNode())) {
                    subSceneGroup.getChildren().remove(ring.getNode());
                }
                storedRings.add(ring);
            }
        }
    }


    /**
     * Remove all rings from the storedRings list, and re-enable them.
     * This should be called only from the application thread.
     */
    public void releaseRings(){
        if (Config.GAME == Config.Game.ULTIMATE_GOAL) {
            while (storedRings.size() > 0) {
                storedRings.get(0).getGeom("ring").enable();
                storedRings.get(0).enable();
                storedRings.remove(0);
            }
        }
    }


    /**
     * Load wobble i onto the bot arm. This is to be called only from the application thread.
     * @param i
     */
    public void loadWobble(int i){
        if (Config.GAME != Config.Game.ULTIMATE_GOAL) return;
        FxBody wobble = UltimateGoalField.getInstance().getWobble(i);

        /*
         * Wobble Rotation and translation relative to grabber
         */
        DMatrix3 rotWobbleRelGrabber = new DMatrix3();
        DRotation.dRSetIdentity(rotWobbleRelGrabber);
        DVector3 transWobbleRelGrabber = new DVector3(0, 20, -15);

        /*
         * Grabber Rotation and translation relative to bot
         */
        double cos = Math.cos(Math.toRadians(grabberRotation));
        double sin = Math.sin(Math.toRadians(grabberRotation));
        DMatrix3 rotGrabberRelBot = new DMatrix3(
                1, 0, 0,
                0, cos, -sin,
                0, sin, cos
        );
        DVector3 transGrabberRelBot = new DVector3(-17.75, 18.75, 14.0);

        /*
         * Wobble Rotation and translation relative to bot
         */
        DMatrix3 rotWobbleRelBot = new DMatrix3();
        rotWobbleRelBot.eqMul(rotGrabberRelBot, rotWobbleRelGrabber);
        DVector3 transWobbleRelBot = new DVector3();
        transWobbleRelBot.eqProd(rotGrabberRelBot, transWobbleRelGrabber);
        transWobbleRelBot.add(transGrabberRelBot);

        /*
         * Wobble Rotation and position in world coordinate system
         */
        DMatrix3C botRot = fxBody.getRotation();
        DMatrix3 wobbleRot = new DMatrix3();
        wobbleRot.eqMul(botRot, rotWobbleRelBot);
        DVector3 wobblePos = new DVector3();
        fxBody.getRelPointPos(transWobbleRelBot, wobblePos);

        wobble.setRotation(wobbleRot);
        wobble.setPosition(wobblePos);

        wobble.updateNodeDisplay();
    }

    public synchronized void updateSensors(){

        //Get current bot position and orientation
        DMatrix3C currentRot = fxBody.getRotation();
        double headingRadians = Math.atan2(currentRot.get10(), currentRot.get00());
        DVector3C currentPos = fxBody.getPosition();
        double x = currentPos.get0();
        double y = currentPos.get1();

        //Based on current bot position and orientation, update sensors
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    public synchronized void updateState(double millis){

        /* Based on KINEMATIC model, determine TENTATIVE changes in X,Y,THETA during next interval
         * Note that these values are based on the AVERAGE SPEED OF EACH WHEEL during the last interval. This is
         * because DcMotorImpl updates POSITION based on the average of initial speed and final speed.
         */

        double[] deltaTicks = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaTicks[i] = motors[i].update(millis);
            w[i] = deltaTicks[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            double wheelRotationDegrees = 360.0 * deltaTicks[i] / motorType.TICKS_PER_ROTATION;
            if (i < 2) {
                w[i] = -w[i];
                wheelRotationDegrees = -wheelRotationDegrees;
            }
            wheelRotations[i] += Math.min(17, Math.max(-17, wheelRotationDegrees));
        }

        double[] robotDeltaPos = new double[] {0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j = 0; j<4; j++){
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        /*
         * Tentative position change in the ROBOT COORDINATE SYSTEM
         */
        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double dHeading = robotDeltaPos[2];

        /*
         * Convert to tentative position change in the WOORLD COORDINATE SYSTEM
         */
        DMatrix3C currentRot = fxBody.getRotation();    //Matrix representing the current orientation in WORLD SYSTEM
        double heading = Math.atan2(currentRot.get10(), currentRot.get00());    //Current heading in WORLD SYSTEM
        double avgHeading = heading + dHeading / 2.0;                    //Tentative average heading during current step
        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);
        double dX = dxR * cosAvg - dyR * sinAvg;        //Tentative change in X-position in WORLD SYSTEM
        double dY = dxR * sinAvg + dyR * cosAvg;        //Tentative change in Y-position in WORLD SYSTEM

        /* Determine the force and torque (WORLD COORDS) that would be required to achieve the changes predicted by
         * the kinematic model.
         *
         * Note: dX, dY, and dHeading are based on average speeds during the interval, so we can calculate force
         * and torque using:
         *
         *       d(Position) = v0*t + 0.5*(F/m)*t*t
         *       d(Heading) = omega0*t + 0.5*(Torque/I)*t*t
         *
         *       or,
         *
         *       F = 2m( d(Position) - v0*t ) / (t*t)
         *       Torque = 2I( d(Heading) - omega0*t ) / (t*t)
         */

        double t = millis / 1000.0;     //Time increment in seconds
        double tSqr = t * t;
        DVector3 deltaPos = new DVector3(dX, dY, 0);    //Vector representing the tentative change in position
        DVector3C vel = fxBody.getLinearVel().clone();      //Robot velocity at the beginning of this step
        ((DVector3)vel).set2(0);                            //Remove any z-component of the beginning velocity
        DVector3 force = deltaPos.reSub(vel.reScale(t)).reScale(2.0 * TOTAL_MASS / tSqr);   //Calculate net force required to achieve the tentative final position
        double angVel = fxBody.getAngularVel().get2();                                      //Angular speed at the beginning of this step
        float torque = (float)(2.0 * TOTAL_Z_INERTIA * (dHeading - angVel*t)/tSqr);         //Calculate net torque required to achieve the tentative final heading

        //Convert the tentative total force to the ROBOT COORDINATE system

        double sinHd = Math.sin(heading);
        double cosHd = Math.cos(heading);

        float fXR = (float)(force.get0()*cosHd + force.get1()*sinHd);
        float fYR = (float)(-force.get0()*sinHd + force.get1()*cosHd);

        VectorF totalBotForces = new VectorF(fXR, fYR, torque, 0);      //Tentative total force & torque on bot in robot coords

        /*
         * Vector to hold the external forces on the robot due to collisions (see below)
         */
        VectorF collisionForces = new VectorF(0,0,0,0);

        /*
          ==========================================  OPTIONAL  ===================================================
          Determine cumulative force & torque on bot, in robot coords, from collisions during preceding world update,
          using the feedbackList. NOTE:  this step is optional; the simulation works quite well without it. This is a more
          physical approach, but it may introduce some problems if the external forces obtained from the physics engine
          are not accurate (As the ODE documentation suggests can occur). If that happens, just remove this step.
          ==========================================================================================================

        for (FeedBack f: feedBackList){
            float fXCR, fYCR;
            if (f.index == 0) {     //Robot is body #1 for this collision
                fXCR = (float)((float)f.fb.f1.get0()*cosHd + (float)f.fb.f1.get1()*sinHd);
                fYCR = (float)(-(float)f.fb.f1.get0()*sinHd + (float)f.fb.f1.get1()*cosHd);
                collisionForces.add(new VectorF(fXCR, fYCR, (float)f.fb.t1.get2(), 0));
            } else {                //Robot is body #2 for this collision
                fXCR = (float)((float)f.fb.f2.get0()*cosHd + (float)f.fb.f2.get1()*sinHd);
                fYCR = (float)(-(float)f.fb.f2.get0()*sinHd + (float)f.fb.f2.get1()*cosHd);
                collisionForces.add(new VectorF(fXCR, fYCR, (float)f.fb.t2.get2(), 0));
            }
        }

        */


        /*
         * We need to clear the feedBack list at this point, whether we are making use of the collision forces or not.
         */
        feedBackList.clear();

        //Frictional force (from floor) on bot, in world coords is tentatively equal to total force minus collision force

        VectorF frictionForces = totalBotForces.subtracted(collisionForces);

        //Determine the forces that would be required on each of the bot's four wheels to achieve
        //the total frictional force and torque predicted by the kinematic model

        VectorF wheel_X_Forces = M_ForceRobotToWheel.multiplied(frictionForces);

        //If any of the wheel forces exceeds the product of muStatic*mass*gravity, reduce the magnitude
        //of that force to muKinetic*mass*gravity, keeping the direction the same

        for (int i=0; i<4; i++){
            float f = wheel_X_Forces.get(i);
            if (Math.abs(f) > MAX_WHEEL_X_FORCE) {
                wheel_X_Forces.put(i, MAX_WHEEL_X_FORCE * Math.signum(f));
            }
        }

        //Based on the adjusted forces at each wheel, determine net frictional force and torque on the bot,
        //Force is in ROBOT COORDINATE system

        frictionForces = M_ForceWheelToRobot.multiplied(wheel_X_Forces);

        //Convert these adjusted friction forces to WORLD COORDINATES and put into the original force DVector3
        force.set0(frictionForces.get(0)*cosHd - frictionForces.get(1)*sinHd);
        force.set1(frictionForces.get(0)*sinHd + frictionForces.get(1)*cosHd);
        force.set2(0);

        /*
         * Apply the adjusted frictional force and torque to the bot.
         *
         * Note:  We are only applying the frictional forces from the floor, NOT the collision forces. The
         *        collision forces will be applied automatically during the next update of the world by the
         *        ODE physics engine.
         */

        fxBody.addForce(force);
        fxBody.addTorque(new DVector3(0, 0, frictionForces.get(2)));

        /*
         * Update intake and shooter motors
         */
        shooterMotor.update(millis);
        intakeMotor.update(millis);

        /*
         * Shooter elevation
         */

//        shooterElevationAngle = 40 - 40 * shooterElevServo.getInternalPosition();

        /*
         * Trigger servo.
         */

        double shooterTrigPos = shooterTrigServo.getInternalPosition();

        if (shooterCocked){
            if (shooterTrigPos > 0.75){
                if (storedRings.size() > 0) shoot();
                shooterCocked = false;
            }
        } else{
            if (shooterTrigServo.getInternalPosition() < 0.25){
                shooterCocked = true;
            }
        }


        /*
         * Hand
         */

        double fingerPos = handServo.getInternalPosition();
        leftHandRotation =  180.0 - 180.0 * fingerPos;
        rightHandRotation = -180.0 + 180.0 * fingerPos;

        /*
         * Arm
         */

        armMotor.update(millis);
        grabberRotation = grabberBaseRotation + armMotor.getActualPosition() * 360.0 / 1120.0;
        if (grabberRotation < -30){
            grabberRotation = -30;
            armMotor.setActualPosition((-30 - grabberBaseRotation) * 1120.0 / 360.0);
        } else if (grabberRotation > 180){
            armMotor.setActualPosition((180 - grabberBaseRotation) * 1120.0 / 360.0);
        }

        /*
         * In the handleContacts method, a wobble can be grabbed only if fingersJustClosed is true.
         */
        fingersJustClosed = fingerPos >=0.99 && priorFingerPos <0.99;
        priorFingerPos = fingerPos;

        /*
         * Handling of a currently grabbed wobble. If fingers have opened even slightly, drop the wobble.
         * Otherwise, set the speed of the wobble motor so that wobble position tracks the position of
         * the grabber arm.
         */
        if (grabbedWobble != null){
            if (fingerPos <0.99){
                wobbleJoint.destroy();
                wobbleMotor.destroy();
                wobbleJoint = null;
                wobbleMotor = null;
                grabbedWobble = null;
            } else {
                double motorAngle = Math.toRadians(grabberRotation);
                double wobbleAngle = wobbleJoint.getAngle();
                double wobbleOffset = AngleUtils.normalizeRadians(motorAngle + wobbleAngle);
                double error = AngleUtils.normalizeRadians(wobbleOffset - wobbleJointBaseOffset);
                wobbleMotor.setParamVel(-100*error/millis);
            }
        }

    }

    private void shoot(){

        if (storedRings.size() == 0) return;
        double shooterMotorPower = shooterMotor.getPower();
        if (shooterMotor.getDirection() == DcMotorSimple.Direction.REVERSE) shooterMotorPower *= -1;
        if (shooterMotorPower <= 0) return;

        /*
         * Get the ring to be shot (an FxBody), re-enable it, and re-enable its DGeom.
         */
        FxBody ring = storedRings.get(0);
        storedRings.remove(0);
        ring.enable();
        ring.getGeom("ring").enable();

        /*
         * Position and orientation of robot in world system
         */
        DMatrix3C botRotation = fxBody.getRotation();
        DVector3C botPosition = fxBody.getPosition();

        /*
         * Position and orientation of axis of rotation of shooter in robot coordinate system
         */
        double cos = Math.cos(Math.toRadians(shooterElevationAngle));
        double sin = Math.sin(Math.toRadians(shooterElevationAngle));
        DMatrix3 shooterRotation = new DMatrix3(-1, 0, 0,
                                                 0, -cos, sin,
                                                 0, sin, cos);
        DVector3 shooterPosition = new DVector3(0, -19, 17);

        /*
         * Position and orientation of axis of rotation of shooter in world coordinate system
         */
        DMatrix3 rotShooterWorld = new DMatrix3();
        rotShooterWorld.eqMul(botRotation, shooterRotation);
        DVector3 posShooterWorld = new DVector3();
        posShooterWorld.eqProd(botRotation, shooterPosition);
        posShooterWorld.add(botPosition);

        /*
         * Calculate, and set, the position and rotation of the ring in world coordinate system at beginning of shot
         */
        DVector3 ringPosition = new DVector3(0, 0, 4);
        DVector3 ringPosWorld = new DVector3();
        ringPosWorld.eqProd(rotShooterWorld, ringPosition);
        ringPosWorld.add(posShooterWorld);
        ring.setRotation(rotShooterWorld);
        ring.setPosition(ringPosWorld);

        /*
         * Initial velocity of ring (before shot), in world coordinates, is the velocity due to robot motion
         */
        DVector3 ringVelocity = new DVector3();
        fxBody.getPointVel(ringPosWorld, ringVelocity);

        /*
         * Added velocity of ring, in world coordinates, due to shooter action
         */
        DVector3 ringAddedVelocity = new DVector3();
        double shootSpeed = shooterMotorPower * 800;
        ringAddedVelocity.eqProd(rotShooterWorld, new DVector3(0, shootSpeed, 0));

        /*
         * Update ring velocity so it is sum of initial and added velocities
         */
        ringVelocity.add(ringAddedVelocity);
        ring.setLinearVel(ringVelocity);

        /*
         * Add spin to ring
         */
        DVector3 spin = new DVector3();
        ring.vectorToWorld(0, 0, 1, spin);
        spin.scale(-shootSpeed / 6.66);
        ring.setAngularVel(spin);

        /*
         * Add ring UI back to the subSceneGroup
         */
        Platform.runLater(
                new Runnable() {
                    @Override
                    public void run() {
                        subSceneGroup.getChildren().add(ring.getNode());
                        ring.updateNodeDisplay();
                    }
                }
        );

    }



    /**
     * Set up FxBody as a single DBody with compound geometry. Interaction between robot components is
     * kinematic, accomplished by changing the offsets of the various DGeom objects belonging to the DBody.
     */
    protected void setUpFxBody(){

        /*
         * Create new FxBody object to represent the chassis. This extends DxBody, and implements DxBody (for physics sim),
         * and contains a Group object for display, and multiple DGeom objects (for collision handling).
         */

        DWorld world = controller.getWorld();
        fxBody = FxBody.newInstance(world, botSpace);
        DBody chassisBody = fxBody;
        DMass chassisMass = OdeHelper.createMass();
        chassisMass.setMass(TOTAL_MASS);
        chassisMass.setI(new DMatrix3(TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA));
        chassisBody.setMass(chassisMass);


        float tetrixWidth = 1.25f * 2.54f;
        float sliderLength = 16f * 2.54f;

        float pltThk = 0.125f * 2.54f;
        float halfPltHt = 2 * 2.54f;
        float halfPltLen1 = 8.5f * 2.54f;
        float halfPltLen2 = 7.5f * 2.54f;
        float pltXOffset1 = 5.5f * 2.54f;
        float pltXOffset2 = 8.5f * 2.54f;
        float pltZOffset = 0.25f * 2.54f;
        double sideBoxLength = 2.0 * halfPltLen1;
        double sideBoxWidth = pltXOffset2 - pltXOffset1 + pltThk;
        double sideBoxHeight = 2.0 * halfPltHt;
        double sideboxXOffset = 0.5 * (pltXOffset1 + pltXOffset2);

        float shortRailLength = 3.125f * 2.54f;
        float longRailLength = 17.125f * 2.54f;
        float shortRailXOffset = 7 * 2.54f;
        float railYOffset = 7.875f * 2.54f;
        float wheelDiam = 4 * 2.54f;
        float wheelWidth = 2 * 2.54f;
        float wheelXOffset = 7 * 2.54f;
        float wheelYOffset = 6.5f * 2.54f;

        /*
         * Create Group for display of robot
         */

        botGroup = new Group();

        /*
         * Group of UI elements for chassis plates
         */
        Group[] plates = new Group[4];
        PhongMaterial plateMaterial = new PhongMaterial(Color.color(0.6, 0.3, 0));
        for (int i=0; i<4; i++){
            plates[i] = Util3D.polygonBox(pltThk, new float[]{halfPltHt ,-halfPltLen1, halfPltHt, halfPltLen1, halfPltHt/2,
                            halfPltLen1, -halfPltHt, halfPltLen2, -halfPltHt, -halfPltLen2, halfPltHt/2, -halfPltLen1},
                    plateMaterial);
            float x = (i<2? -1.0f : 1.0f) * (i%2 == 0? pltXOffset2:pltXOffset1);
            plates[i].getTransforms().addAll(new Translate(x, 0, pltZOffset), new Rotate(-90, Rotate.Y_AXIS));
        }

        /*
         * Groups for the UI elements for the tetrix rails at the back and front of the robot
         */
        Group frontLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontRightRail.getTransforms().addAll(new Translate(shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRail = Parts.tetrixBox(longRailLength, tetrixWidth, 2*tetrixWidth, tetrixWidth);
        frontRail.getTransforms().addAll(new Translate(0, railYOffset, halfPltHt+pltZOffset+2*tetrixWidth));
        Group backLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backRightRail.getTransforms().addAll(new Translate(shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRail = Parts.tetrixBox(longRailLength, tetrixWidth, 2*tetrixWidth, tetrixWidth);
        backRail.getTransforms().addAll(new Translate(0, -railYOffset, halfPltHt+pltZOffset+2*tetrixWidth));

        /*
         * Add plates and rail groups to botGroup
         */
        botGroup.getChildren().addAll(plates);
        botGroup.getChildren().addAll(frontLeftRail, frontRightRail, frontRail, backLeftRail, backRightRail, backRail);

        /*
         * Groups for the UI elements for the mecanum wheels
         */
        Group[] wheels = new Group[4];
        for (int i=0; i<4; i++){
            wheels[i] = Parts.mecanumWheel(wheelDiam, wheelWidth, i);
            wheels[i].setRotationAxis(new Point3D(0, 0, 1));
            wheels[i].setRotate(90);
            wheels[i].setTranslateX(i<2? -wheelXOffset : wheelXOffset);
            wheels[i].setTranslateY(i==0 || i==3? -wheelYOffset : wheelYOffset);
            wheels[i].getTransforms().add(wheelRotates[i]);
        }

        botGroup.getChildren().addAll(wheels);

        /*
         * Establish botGroup as the node belonging to fxBody.
         */

        fxBody.setNode(botGroup, false);

        /*
         * Generate DGeom objects (they happen to all be boxes) for chassis collision handling
         */

        DBox rightSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox frontRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 1.8*tetrixWidth);
        DBox leftBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox backRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 1.8*tetrixWidth);

        DTriMeshData botBottomData = FxBodyHelper.getParametricTriMeshData(1, -1, -1, 1, 5, 5,
                false, false, new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s * halfPltLen1;
                    }

                    @Override
                    public float y(float s, float t) {
                        return t * halfPltLen1;
                    }

                    @Override
                    public float z(float s, float t) {
                        return 0;
                    }
                });

        DTriMesh botBottomMesh = OdeHelper.createTriMesh(botSpace, botBottomData, null, null, null);
        botBottomMesh.setData("Bot Bottom Mesh");

        /*
         * Add the chassis DGeom objects to fxBody, with appropriate offsets
         */

        fxBody.addGeom(rightSideBox, sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftSideBox, -sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftFrontRailBox, -shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightFrontRailBox, shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(frontRailBox, 0, railYOffset, pltZOffset+halfPltHt+2*tetrixWidth);
        fxBody.addGeom(leftBackRailBox, -shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightBackRailBox, shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(backRailBox, 0, -railYOffset, pltZOffset+halfPltHt+2*tetrixWidth);
        fxBody.addGeom(botBottomMesh, 0, 0, -halfPltHt);

        /*
         * Ring Intake: four green cylinders for UI, but a single long cylinder for collision handling
         */

        PhongMaterial intakeMaterial = new PhongMaterial(Color.GREEN);
        Group intakeGroup = new Group();
        for (int i=0; i<4; i++){
            Cylinder intakeWheelCyl = new Cylinder(2, 0.8*pltXOffset1/2);
            intakeWheelCyl.getTransforms().addAll(new Translate((-1.5+i)*pltXOffset1/2, halfPltLen2, -halfPltHt+pltZOffset+2),
                    new Rotate(90, Rotate.Z_AXIS));
            intakeWheelCyl.setMaterial(intakeMaterial);
            intakeGroup.getChildren().add(intakeWheelCyl);
        }
        botGroup.getChildren().add(intakeGroup);
        DGeom intakeGeom = OdeHelper.createCylinder(botSpace, 2, 1.6*pltXOffset1);
        DMatrix3 R = new DMatrix3();
        DRotation.dRFromAxisAndAngle(R, 0, 1, 0, Math.PI/2);
        fxBody.addGeom(intakeGeom, 0, halfPltLen2, -halfPltHt+pltZOffset+2, R);

        /*
         * Shooter
         *
         * A GroupWithDGeoms object composed of multiple ShapeWithDGeom objects; these belong to classes that
         * extend Shape3D, and each possesses a DGeom object.
         *
         * When the shooter is moved relative to the robot (and also after initial creation), its updateGeomOffsets()
         * method must be called so that the DGeom objects track this movement.
         *
         */

        PhongMaterial shooterMaterial = new PhongMaterial(Color.AQUA);
        PhongMaterial shooterWheelMaterial = new PhongMaterial(Color.BLUE);
        PhongMaterial shooterRailMaterial = new PhongMaterial(Color.CORAL);
        shooter = new GroupWithDGeoms();
        BoxWithDGeom shooterBed = new BoxWithDGeom(15, 30, 1.25, fxBody, "Shooter Bed");
        BoxWithDGeom leftShooterRail = new BoxWithDGeom(1.25, 15, 5, fxBody, "Left Shooter Rail");
        BoxWithDGeom rightShooterRail = new BoxWithDGeom(1.25, 30, 5, fxBody, "Right Shooter Rail");
        BoxWithDGeom backShooterRail = new BoxWithDGeom(12.5, 1.25, 5, fxBody, "Back Shooter Rail");
        BoxWithDGeom shooterTop = new BoxWithDGeom(15, 15, 1.25, fxBody, "Shooter Top");
        CylWithDGeom shooterWheel = new CylWithDGeom(4.25, 2, fxBody, "Shooter Wheel");
        leftShooterRail.getTransforms().add(new Translate(-6.875, -7.5, 2.75));
        rightShooterRail.getTransforms().add(new Translate(6.875, 0, 2.75));
        backShooterRail.getTransforms().add(new Translate(0, -14.375, 2.75));
        shooterTop.getTransforms().add(new Translate(0, -7.5, 6.25));
        shooterWheel.getTransforms().addAll(new Translate(-11, 8.75, 1.5), new Rotate(90, Rotate.X_AXIS));
        shooterBed.setMaterial(shooterMaterial);
        leftShooterRail.setMaterial(shooterRailMaterial);
        rightShooterRail.setMaterial(shooterRailMaterial);
        backShooterRail.setMaterial(shooterRailMaterial);
        shooterTop.setMaterial(shooterRailMaterial);
        shooterWheel.setMaterial(shooterWheelMaterial);
        shooter.getTransforms().addAll(new Rotate(180, Rotate.Z_AXIS), new Translate(0, 6, 17), shooterElevationRotate);
        shooter.getChildren().addAll(shooterBed, leftShooterRail, rightShooterRail, backShooterRail, shooterTop, shooterWheel);
        botGroup.getChildren().add(shooter);
        shooter.updateGeomOffsets();

        /*
         * Back wedge keeps rings from entering bot from behind, and also keeps wobbles from getting hung
         * up inside bot
         */
        GroupWithDGeoms backWedge = new GroupWithDGeoms();
        BoxWithDGeom box1 = new BoxWithDGeom(2*pltXOffset1, 2, 2, fxBody);
        box1.getTransforms().add(new Translate(0, -2, -2));
        BoxWithDGeom box2 = new BoxWithDGeom(2*pltXOffset1, 2, 4, fxBody);
        box2.getTransforms().add(new Translate(0, 0, -1));
        BoxWithDGeom box3 = new BoxWithDGeom(2*pltXOffset1, 2, 6, fxBody);
        box3.getTransforms().add(new Translate(0, 2, 0));
        backWedge.getChildren().addAll(box1, box2, box3);
        for (Node n: backWedge.getChildren()) ((Shape3D)n).setMaterial(new PhongMaterial(Color.TAN));
        backWedge.getTransforms().add(new Translate(0, -15, -1));
        backWedge.updateGeomOffsets();
        botGroup.getChildren().add(backWedge);

        /*
         * Wobble grabber.
         *
         * A GroupWithDGeoms
         */
        PhongMaterial grabberMaterial = new PhongMaterial(Color.CORAL);
        PhongMaterial handMaterial = new PhongMaterial(Color.BROWN);
        grabber = new GroupWithDGeoms();
        BoxWithDGeom arm = new BoxWithDGeom(7.5, 20, 3.0, fxBody);
        GroupWithDGeoms leftHand = new GroupWithDGeoms();
        BoxWithDGeom leftMetacarpal = new BoxWithDGeom(2.5, 2.5, 3.0, fxBody);
        leftMetacarpal.getTransforms().add(new Translate(1.25, 1.25, 0));
        BoxWithDGeom leftPhalanx = new BoxWithDGeom(3.75, 2.5, 3.0, fxBody);
        leftPhalanx.getTransforms().add(new Translate(3.75/2, 3.75, 0));
        leftHand.getChildren().addAll(leftMetacarpal, leftPhalanx);
        GroupWithDGeoms rightHand = new GroupWithDGeoms();
        BoxWithDGeom rightMetacarpal = new BoxWithDGeom(2.5, 2.5, 3.0, fxBody);
        rightMetacarpal.getTransforms().add(new Translate(-1.25, 1.25, 0));
        BoxWithDGeom rightPhalanx = new BoxWithDGeom(3.75, 2.5, 3.0, fxBody);
        rightPhalanx.getTransforms().add(new Translate(-3.75/2, 3.75, 0));
        rightHand.getChildren().addAll(rightMetacarpal, rightPhalanx);
        arm.getTransforms().add(new Translate(0, 10, 0));
        leftHand.getTransforms().addAll(new Translate(-3.75, 20, 0), leftHandRotate);
        rightHand.getTransforms().addAll(new Translate(3.75, 20, 0), rightHandRotate);
        grabber.getChildren().addAll(arm, leftHand, rightHand);
        grabber.getTransforms().addAll(new Translate(-17.75, 18.75, 14), grabberRotate);
        arm.setMaterial(grabberMaterial);
        leftMetacarpal.setMaterial(handMaterial);
        leftPhalanx.setMaterial(handMaterial);
        rightMetacarpal.setMaterial(handMaterial);
        rightPhalanx.setMaterial(handMaterial);
        grabber.updateGeomOffsets();
        botGroup.getChildren().add(grabber);

        /*
         * Set the value of the zBase field of the superclass (VirtualBot).
         *
         * This is the height to which the bot must be raised so that the wheels rest on the field surface.
         */
        zBase = 5.08;

        /*
         * Set collision handling bits for the various bot DGeom elements
         */
        fxBody.setCategoryBits(CBits.BOT);                          //  The default
        fxBody.setCollideBits(0xFFFFFFFFFFFFFFFFL);                 //  Bot collides with everything by default
        botBottomMesh.setCategoryBits(CBits.BOT_BOTTOM);
        botBottomMesh.setCollideBits(CBits.FLOOR);                  //  Bot bottom collides ONLY with the floor
        intakeGeom.setCategoryBits(CBits.BOT_RING_INTAKE);
        shooterBed.getDGeom().setCategoryBits(CBits.SHOOTER);
        leftShooterRail.getDGeom().setCategoryBits(CBits.SHOOTER);
        rightShooterRail.getDGeom().setCategoryBits(CBits.SHOOTER);
        shooterTop.getDGeom().setCategoryBits(CBits.SHOOTER);
        backShooterRail.getDGeom().setCategoryBits(CBits.SHOOTER);
        shooterWheel.getDGeom().setCategoryBits(CBits.SHOOTER);
        leftMetacarpal.getDGeom().setCategoryBits(CBits.HAND);
        rightMetacarpal.getDGeom().setCategoryBits(CBits.HAND);
        arm.getDGeom().setCategoryBits(CBits.ARM);
        leftPhalanx.getDGeom().setCategoryBits(CBits.HAND);
        rightPhalanx.getDGeom().setCategoryBits(CBits.HAND);
    }


    /**
     * Override the updateDisplay() method. The superclass method handles positioning bot UI on the field.
     * Bot mechanisms are handles by the overridden method.
     */
    @Override
    public synchronized void updateDisplay(){
        super.updateDisplay();
        for (int i=0; i<4; i++) wheelRotates[i].setAngle(wheelRotations[i]);
        shooterElevationRotate.setAngle(shooterElevationAngle);
        grabberRotate.setAngle(grabberRotation);
        leftHandRotate.setAngle(leftHandRotation);
        rightHandRotate.setAngle(rightHandRotation);
        grabber.updateGeomOffsets();
        shooter.updateGeomOffsets();
    }


    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        imu.close();
        if (wobbleJoint != null) {
            wobbleJoint.destroy();
            wobbleJoint = null;
        }
        if (wobbleMotor != null) {
            wobbleMotor.destroy();
            wobbleMotor = null;
        }
        grabbedWobble = null;
        handServo.setDirection(Servo.Direction.FORWARD);
        handServo.setPosition(1);
        leftHandRotation = 0;
        rightHandRotation = 0;
        priorFingerPos = 0;
        fingersJustClosed = true;
        grabberBaseRotation = grabberRotation;
        armMotor.stopAndReset();
        Platform.runLater(
                new Runnable() {
                    @Override
                    public void run() {
                        updateDisplay();
                    }
                }
        );
    }

    public synchronized void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup){
        long o1CBits = o1.getCategoryBits();
        long o2CBits = o2.getCategoryBits();

        boolean o1Arm = (o1CBits & CBits.ARM) != 0,
                o2Arm = (o2CBits & CBits.ARM) != 0,
                o1Hand = (o1CBits & CBits.HAND) != 0,
                o2Hand = (o2CBits & CBits.HAND) != 0,
                o1BotBottom = (o1CBits & CBits.BOT_BOTTOM) != 0,
                o2BotBottom = (o2CBits & CBits.BOT_BOTTOM) != 0,
                o1Intake = (o1CBits & CBits.BOT_RING_INTAKE) != 0,
                o2Intake = (o2CBits & CBits.BOT_RING_INTAKE) != 0,
                o1Ring = (o1CBits & CBits.RINGS) != 0,
                o2Ring = (o2CBits & CBits.RINGS) != 0,
                o1Shooter = (o1CBits & CBits.SHOOTER) != 0,
                o2Shooter = (o2CBits & CBits.SHOOTER) != 0,
                o1Wobble = (o1CBits & CBits.WOBBLES) != 0,
                o2Wobble = (o2CBits & CBits.WOBBLES) != 0;


        /*
         * If this is a collision between the intake and a ring (and the bot is currently storing less than 3 rings,
         * disable the ring, remove it from the UI, and add it to the "store" of rings on the bot.
         */
        if ((o1Ring && o2Intake || o2Ring && o1Intake) && storedRings.size() < 3
                && (intakeMotor.getPower() > 0.2 && intakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD
                || intakeMotor.getPower() < -0.2 && intakeMotor.getDirection() == DcMotorSimple.Direction.REVERSE)){
            FxBody ring = o1Ring? (FxBody)o1.getBody() : (FxBody)o2.getBody();
            if (storedRings.contains(ring)) return;
            ring.getGeom("ring").disable();
            ring.disable();
            storedRings.add(ring);
            Platform.runLater(
                    new Runnable() {
                        @Override
                        public void run() {
                            if (subSceneGroup.getChildren().contains(ring.getNode())){
                                subSceneGroup.getChildren().remove(ring.getNode());
                            }
                        }
                    }
            );
            return;
        }

        for (int i=0; i<numContacts; i++)
        {
            DContact contact = contacts.get(i);

            if (o1Arm && o2Wobble || o2Arm && o1Wobble){

                /*
                 * Interaction between arm and wobble
                 *
                 * If wobble is contacting DISTAL (+y-facing) end of the arm, AND the hand has just closed, AND
                 * there currently is no grabbed wobble, then grab this wobble as follows: create a hinge joint
                 * between the bot and the wobble, as well as an angular motor joint.
                 *
                 * If there is no grabbed wobble but the other conditions for grabbing are not met, then the arm
                 * and wobble collide in a normal manner.
                 *
                 * If there is already a grabbed wobble, then no interaction between arm and currently-colliding
                 * wobble.
                 */

                double armElev = Math.toRadians(grabberRotation);
                DVector3 armAxisBot = new DVector3(0, Math.cos(armElev), Math.sin(armElev));
                DVector3 armAxisWorld = new DVector3();
                fxBody.vectorToWorld(armAxisBot, armAxisWorld);
                double dot = armAxisWorld.dot(contact.geom.normal);
                if (o1Arm) dot *= -1;

                if (dot >0.8 && fingersJustClosed && grabbedWobble==null) {       //All conditions for grabbing are met
                    grabbedWobble = o1Wobble? (FxBody)o1.getBody() : (FxBody)o2.getBody();
                    wobbleJoint = OdeHelper.createHingeJoint(controller.getWorld());
                    wobbleJoint.attach(fxBody, grabbedWobble);
                    DVector3 anchorWorldPos = new DVector3();
                    fxBody.getRelPointPos(-17.75, 18.75, 14, anchorWorldPos);
                    wobbleJoint.setAnchor(anchorWorldPos);
                    DVector3 axisXWorld = new DVector3();
                    fxBody.vectorToWorld(1, 0, 0, axisXWorld);
                    wobbleJoint.setAxis(axisXWorld);
                    wobbleJointBaseOffset = AngleUtils.normalizeRadians(Math.toRadians(grabberRotation) + wobbleJoint.getAngle());
                    wobbleMotor = OdeHelper.createAMotorJoint(controller.getWorld());
                    wobbleMotor.attach(fxBody, grabbedWobble);
                    wobbleMotor.setMode(DAMotorJoint.AMotorMode.dAMotorUser);
                    wobbleMotor.setNumAxes(1);
                    wobbleMotor.setAxis(0, 1, axisXWorld);
                    wobbleMotor.setParamFMax(dInfinity);
                    wobbleMotor.setParamVel(0);
                    continue;
                } else if (grabbedWobble == null){      //No grabbed wobble, but conditions for grabbing not met
                    contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                    contact.surface.mu = 0.5;
                    contact.surface.soft_cfm = 0.00000001;
                    contact.surface.soft_erp = 0.2;
                    contact.surface.bounce = 0.3;
                    contact.surface.bounce_vel = 10;
                } else {        //There is already a grabbed wobble
                    continue;
                }

            } else if (o1Hand && o2Wobble || o2Hand && o1Wobble){

                /*
                 * Ignore collisions between Hand and Wobble.
                 */

                continue;
            } else if (o1BotBottom || o2BotBottom) {
                /*
                 * NOTE: Contacts involving the bottom surface of the bot are used ONLY to support the bot.
                 *       There is no friction for these contacts. As far as the physics engine is concerned,
                 *       the bot is gliding without friction over the field. Kinematics of robot motion on
                 *       the field are handled in the updateState() method, and include an estimate of the effects
                 *       of friction.
                 */
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact.surface.mu = 0;
                contact.surface.soft_cfm = 0.00000001;
                contact.surface.soft_erp = 0.2;
                contact.surface.bounce = 0.3;
                contact.surface.bounce_vel = 10;
            } else if (o1Ring && o2Shooter || o2Ring && o1Shooter) {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact.surface.mu = 0;
                contact.surface.soft_cfm = 0.00000001;
                contact.surface.soft_erp = 0.2;
                contact.surface.bounce = 0.3;
                contact.surface.bounce_vel = 10;
            } else {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact.surface.mu = 0.5;
                contact.surface.soft_cfm = 0.00000001;
                contact.surface.soft_erp = 0.2;
                contact.surface.bounce = 0.3;
                contact.surface.bounce_vel = 10;
            }
            DJoint c = OdeHelper.createContactJoint (controller.getWorld(),contactGroup,contact);
            c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody());

            /*
             * Create a new DJointFeedback object and assign it to this contact joint. It will be populated with the
             * force and torque attributable to this contact joint during the next world update. From this, create a
             * Feedback object that includes the DJointFeedback object as well as an index. The index indicates whether
             * the robot is the first or second body participating in this joint. Add the Feedback object to the
             * feedBackList. The list will be used to determine cumulative force and torque on the robot from
             * collisions when doing the next iteration of robot kinematics (i.e., call to updateState).
             */
            DJoint.DJointFeedback fb = new DJoint.DJointFeedback();
            if (c.getBody(0) == fxBody) {
                c.setFeedback(fb);
                feedBackList.add(new FeedBack(fb, 0));
            } else if (c.getBody(1) == fxBody){
                c.setFeedback(fb);
                feedBackList.add(new FeedBack(fb, 1));
            }
        }

    }


}
