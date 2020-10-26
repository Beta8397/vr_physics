package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
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
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;
import org.ode4j.math.*;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

import static org.ode4j.ode.OdeConstants.*;

/**
 *  Team Beta 8397 specific robot configuration
 *
 */
@BotConfig(name = "Beta Bot")
public class BetaBot extends VirtualBot {

    private final float TOTAL_MASS = 15000;  //mg
    private final float TOTAL_Z_INERTIA = 5000000f; //gm*cm2
    private final float FIELD_FRICTION_COEFF = 1.0f;
    private final float GRAVITY = 980f; // cm/s2
    //Max possible force (in Robot-X direction) at any wheel (each wheel gets 1/4 of robot weight)
    private final float MAX_WHEEL_X_FORCE = TOTAL_MASS * GRAVITY * FIELD_FRICTION_COEFF / (4.0f * (float)Math.sqrt(2));

    public final MotorType motorType = MotorType.NeverestOrbital20;
    private DcMotorImpl[] motors = null;
    private DcMotorImpl liftMotor = null;
    private BNO055EnhancedImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl handServo = null;
    private CRServoImpl sliderCRServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;
    private DcMotorImpl leftIntakeMotor = null;
    private DcMotorImpl rightIntakeMotor = null;

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private double[][] tWR; //Transform from wheel motion to robot motion

    GeneralMatrixF  M_ForceWheelToRobot;
    MatrixF M_ForceRobotToWheel;

    GroupWithDGeoms lift;

    Translate rightHandTranslate = new Translate(0, 0, 0);
    Translate leftHandTranslate = new Translate(0, 0, 0);
    Translate sliderTranslate = new Translate(0, -10, 0);
    Translate[] liftTranslates = new Translate[]{new Translate(0, 0, 0), new Translate(0, 0, 0), new Translate(0, 0, 0)};
    Rotate[] wheelRotates = new Rotate[] {new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS)};

    double handRotation = 0;
    double handTranslation = 0;
    double sliderTranslation = -10;
    double liftElevation = 0;
    double[] wheelRotations = new double[]{0,0,0,0};
    double verticalLiftSpeed = 0;
    double sliderSpeed = 0;
    double rightIntakeSpeed = 0;
    double leftIntakeSpeed = 0;


    @Override
    public void init(){
        super.init();

        hardwareMap.setActive(true);
        motors = new DcMotorImpl[]{
                (DcMotorImpl)hardwareMap.dcMotor.get("back_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_right_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("back_right_motor")
        };

        liftMotor = (DcMotorImpl)hardwareMap.dcMotor.get("lift_motor");

        leftIntakeMotor = (DcMotorImpl)hardwareMap.dcMotor.get("left_intake_motor");
        rightIntakeMotor = (DcMotorImpl)hardwareMap.dcMotor.get("right_intake_motor");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055EnhancedImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");
        sliderCRServo = (CRServoImpl)hardwareMap.crservo.get("slider_crservo");
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

    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorImpl(motorType));
        hardwareMap.put("lift_motor", new DcMotorImpl(MotorType.Neverest40, false, false));
        hardwareMap.put("left_intake_motor", new DcMotorImpl(MotorType.Neverest40, false, false));
        hardwareMap.put("right_intake_motor", new DcMotorImpl(MotorType.Neverest40, false, false));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055EnhancedImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("hand_servo", new ServoImpl());
        hardwareMap.put("slider_crservo", new CRServoImpl(720));
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
        System.out.printf("DH = %.12f  AngVel = %.5f  \n\n", dHeading, angVel);

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
        System.out.println("Added torque: " + frictionForces.get(2));

        /**
         * Update state of robot accessories
         */


        double newHandTranslation = handServo.getInternalPosition() * 4.0;
        handTranslation = newHandTranslation;

        double oldSliderTranslation = sliderTranslation;
        double oldLiftElevation = liftElevation;

        double newSliderTranslation = sliderTranslation + sliderCRServo.updatePositionDegrees(millis) * 10.0 / 360.0;
        double deltaLiftMotor = liftMotor.update(millis);
        double newLiftElevation = liftElevation + deltaLiftMotor * 20.0 / 1120.0;
        double minElev = 0;

        if (newLiftElevation >= 14.1 || newSliderTranslation <= -5.5 || newSliderTranslation >= 10){
            sliderTranslation = Math.min(16, Math.max(-10, newSliderTranslation));
        } else if (sliderTranslation <= -5.5){
            sliderTranslation = Math.min(-5.5, Math.max(-10, newSliderTranslation));
        } else if (sliderTranslation >= 10) {
            sliderTranslation = Math.min(16, Math.max(10, newSliderTranslation));
        } else {
            sliderTranslation = Math.min(16, Math.max(-10, newSliderTranslation));
            minElev = 14.1;
        }

        sliderSpeed = (sliderTranslation - oldSliderTranslation) / millis;

        if (newLiftElevation > 91){
            if (Math.abs(newLiftElevation - liftElevation) < 0.0001) liftMotor.adjustActualPosition(-deltaLiftMotor);
            else {
                liftMotor.adjustActualPosition(-deltaLiftMotor *(newLiftElevation - 91)/(newLiftElevation - liftElevation));
                liftElevation = 91;
            }
        } else if (newLiftElevation < minElev) {
            if (Math.abs(newLiftElevation - liftElevation) < 0.0001) liftMotor.adjustActualPosition(-deltaLiftMotor);
            else {
                liftMotor.adjustActualPosition(-deltaLiftMotor * (newLiftElevation - minElev) / (newLiftElevation - liftElevation));
                liftElevation = minElev;
            }
        } else {
            liftElevation = newLiftElevation;
        }

        verticalLiftSpeed =  1000.0 * (liftElevation - oldLiftElevation) / millis;
        sliderSpeed = 1000.0 * (sliderTranslation - oldSliderTranslation) / millis;

        rightIntakeSpeed = 1000.0 * rightIntakeMotor.update(millis) * 50.0/(1120.0 * millis);
        leftIntakeSpeed = 1000.0* leftIntakeMotor.update(millis) * 50.0 / (1120.0 *  millis);

    }



    /**
     * Set up FxBody2 as a single DBody with compound geometry. Interaction between robot components is
     * kinematic, accomplished by changing the offsets of the various DGeom objects belonging to the DBody.
     *
     * Disadvantage: Contact joints between the component DGeoms and external objects are between the DBody objects,
     * so friction doesn't work to move (e.g., lift) an external object when the offset of the contacting DGeom
     * is moved.
     *
     * Possible solution: Add additional DBody objects (e.g., for the hand) that are kinematic, and
     * track the position of the kinematically-controlled components.
     */
    protected void setUpFxBody(){

        //Create new FxBody2 object to represent the chassis. This will contain the DBody (for physics sim),
        //a Group object for display, and multiple DGeom objects (for collision handling). It will
        //also have children--these will be other FxBody2 objects that represent robot components other than
        //the chassis.
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

        float liftThk = 0.6f*2.54f;
        float liftHt = 13f * 2.54f;
        float liftXOffset = 5.8625f * 2.54f;
        float liftBaseYOffset = 2.0f * 2.54f;
        float liftZOffset = 4.75f * 2.54f;

        float crossBarZOffset = liftHt/2 - liftThk/2;
        float crossBarLength = 2 * liftXOffset - liftThk;

        PhongMaterial liftMaterial0 = new PhongMaterial(Color.color(0.6, 0.6, 0.6));
        PhongMaterial liftMaterial1 = new PhongMaterial(Color.color(0.8, 0.8, 0.8));

        //Create Group for display of chassis

        Group botGroup = new Group();

        Group[] plates = new Group[4];
        PhongMaterial plateMaterial = new PhongMaterial(Color.color(0.6, 0.3, 0));
        for (int i=0; i<4; i++){
            plates[i] = Util3D.polygonBox(pltThk, new float[]{halfPltHt ,-halfPltLen1, halfPltHt, halfPltLen1, halfPltHt/2,
                            halfPltLen1, -halfPltHt, halfPltLen2, -halfPltHt, -halfPltLen2, halfPltHt/2, -halfPltLen1},
                    plateMaterial);
            float x = (i<2? -1.0f : 1.0f) * (i%2 == 0? pltXOffset2:pltXOffset1);
            plates[i].getTransforms().addAll(new Translate(x, 0, pltZOffset), new Rotate(-90, Rotate.Y_AXIS));
        }

        Group frontLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontRightRail.getTransforms().addAll(new Translate(shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRail = Parts.tetrixBox(longRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontRail.getTransforms().addAll(new Translate(0, railYOffset, halfPltHt+pltZOffset+1.5*tetrixWidth));
        Group backLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backRightRail.getTransforms().addAll(new Translate(shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRail = Parts.tetrixBox(longRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backRail.getTransforms().addAll(new Translate(0, -railYOffset, halfPltHt+pltZOffset+1.5*tetrixWidth));

        botGroup.getChildren().addAll(plates);
        botGroup.getChildren().addAll(frontLeftRail, frontRightRail, frontRail, backLeftRail, backRightRail, backRail);

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

        PhongMaterial slideMaterial = Util3D.imageMaterial("/virtual_robot/assets/rev_slide.jpg");

        Box leftLiftBase = new Box(liftThk, liftThk, liftHt);
        leftLiftBase.setMaterial(liftMaterial0);
        leftLiftBase.getTransforms().addAll(new Translate(-liftXOffset, liftBaseYOffset, liftZOffset));
        Box rightLiftBase = new Box(liftThk, liftThk, liftHt);
        rightLiftBase.setMaterial(liftMaterial0);
        rightLiftBase.getTransforms().addAll(new Translate(liftXOffset, liftBaseYOffset, liftZOffset));
        botGroup.getChildren().addAll(leftLiftBase, rightLiftBase);

        //For display purposes, set the Node object of fxBody to the display group

        fxBody.setNode(botGroup, false);

        //Generate DGeom objects (they happen to all be boxes) for chassis collision handling

        DBox rightSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox frontRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox leftBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox backRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox leftLiftBaseBox = OdeHelper.createBox(liftThk, liftThk, liftHt);
        DBox rightLiftBaseBox = OdeHelper.createBox(liftThk, liftThk, liftHt);

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

        //Add the chassis DGeom objects to fxBody, with appropriate offsets

        fxBody.addGeom(rightSideBox, sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftSideBox, -sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftFrontRailBox, -shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightFrontRailBox, shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(frontRailBox, 0, railYOffset, pltZOffset+halfPltHt+1.5*tetrixWidth);
        fxBody.addGeom(leftBackRailBox, -shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightBackRailBox, shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(backRailBox, 0, -railYOffset, pltZOffset+halfPltHt+1.5*tetrixWidth);
        fxBody.addGeom(leftLiftBaseBox, -liftXOffset, liftBaseYOffset, liftZOffset);
        fxBody.addGeom(rightLiftBaseBox, liftXOffset, liftBaseYOffset, liftZOffset);
        fxBody.addGeom(botBottomMesh, 0, 0, -halfPltHt);


        lift = new GroupWithDGeoms("lift");
        GroupWithDGeoms[] liftStages = new GroupWithDGeoms[3];

        for (int i=0; i<3; i++){
            liftStages[i] = new GroupWithDGeoms("lift stage " + i);
            BoxWithDGeom left = new BoxWithDGeom(liftThk, liftThk, liftHt, fxBody, "  left box");
            left.getTransforms().addAll(new Translate(-liftXOffset, 0, 0));
            BoxWithDGeom right = new BoxWithDGeom(liftThk, liftThk, liftHt, fxBody, "  right box");
            right.getTransforms().addAll(new Translate(liftXOffset, 0, 0));
            left.setMaterial(i%2 == 0? liftMaterial1 : liftMaterial0);
            right.setMaterial(i%2 == 0? liftMaterial1 : liftMaterial0);
            liftStages[i].getChildren().addAll(left, right);
            liftStages[i].getTransforms().addAll(liftTranslates[i], new Translate(0, liftThk, 0));
            if (i>0) liftStages[i-1].getChildren().add(liftStages[i]);
        }

        lift.getChildren().add(liftStages[0]);

        BoxWithDGeom crossBar = new BoxWithDGeom(crossBarLength, liftThk, liftThk, fxBody, "  cross bar");
        crossBar.setMaterial(liftMaterial0);
        crossBar.getTransforms().addAll(new Translate(0, 0, crossBarZOffset));
        Box box = new Box(7, 3, 3);
        box.setMaterial(new PhongMaterial(Color.color(0.3, 0.3, 0.3)));
        box.getTransforms().add(new Translate(0, 0, crossBarZOffset-liftThk));
        liftStages[2].getChildren().addAll(crossBar, box);

        GroupWithDGeoms slideGroup = new GroupWithDGeoms("slideGroup");

        BoxWithDGeom slider = new BoxWithDGeom(tetrixWidth, sliderLength, tetrixWidth, fxBody, "  slider");
        slider.setMaterial(liftMaterial1);
        BoxWithDGeom boxWithDGeom = new BoxWithDGeom(3.2, 6.4, 10, fxBody, "  slider box 1");
        boxWithDGeom.setMaterial(new PhongMaterial(Color.color(0.3, 0.3, 0.3)));
        boxWithDGeom.getTransforms().add(new Translate(0, 0.2*sliderLength, -tetrixWidth/2 - 5));
        slideGroup.getChildren().addAll(slider, boxWithDGeom);
        boxWithDGeom = new BoxWithDGeom(16,12.5, 0.4, fxBody, "  slider box 2");
        boxWithDGeom.setMaterial(new PhongMaterial(Color.color(0.6, 0.6, 0, 0.5)));
        boxWithDGeom.getTransforms().add(new Translate(0, 0.2*sliderLength, -tetrixWidth/2 - 10));
        slideGroup.getChildren().add(boxWithDGeom);
        BoxWithDGeom leftHand = new BoxWithDGeom(0.4, 12.5, 16, fxBody, OdeHelper.createBox(1, 12.5, 16));
        leftHand.setRelGeomOffset(new Translate(0.3, 0, 0));
        leftHand.setMaterial(new PhongMaterial(Color.color(0.6, 0, 0.6, 0.5)));
        leftHand.getTransforms().addAll(new Translate(-8, 0.2*sliderLength, -tetrixWidth/2-18), leftHandTranslate);
        slideGroup.getChildren().add(leftHand);
        BoxWithDGeom rightHand = new BoxWithDGeom(0.4, 12.5, 16, fxBody, OdeHelper.createBox(1, 12.5, 16));
        rightHand.setRelGeomOffset(new Translate(-0.3, 0, 0));
        rightHand.setMaterial(new PhongMaterial(Color.color(0.6, 0, 0.6, 0.5)));
        rightHand.getTransforms().addAll(new Translate(8, 0.2*sliderLength, -tetrixWidth/2-18), rightHandTranslate);
        slideGroup.getChildren().add(rightHand);
        slideGroup.getTransforms().addAll(sliderTranslate, new Translate(0, 0, liftHt/2  - 3));

        DGeom rightHandGeom = rightHand.getDGeom();
        DGeom leftHandGeom = leftHand.getDGeom();

        liftStages[2].getChildren().add(slideGroup);

        lift.getTransforms().add(new Translate(0, liftBaseYOffset, liftHt/2 - halfPltHt + pltZOffset));

        botGroup.getChildren().add(lift);

        lift.updateGeomOffsets();

        GroupWithDGeoms intakeGroup = new GroupWithDGeoms("Intake Group");
        GroupWithDGeoms leftIntakeGroup = new GroupWithDGeoms("Left Intake Group");
        GroupWithDGeoms rightIntakeGroup = new GroupWithDGeoms("Right Intake Group");
        PhongMaterial intakeMaterial = new PhongMaterial(Color.GREEN);

        for (int i = 0; i<3; i++){
            CylWithDGeom leftIntakeWheel = new CylWithDGeom(2.5, 5, fxBody,
                    "Left Intake Wheel" + i, OdeHelper.createCylinder(botSpace, 2.5, 5));
            leftIntakeWheel.setMaterial(intakeMaterial);
            leftIntakeWheel.getTransforms().addAll(new Translate(-7.5, (1-i)*6.5, 0), new Rotate(90, Rotate.X_AXIS));
            leftIntakeGroup.getChildren().add(leftIntakeWheel);
            CylWithDGeom rightIntakeWheel = new CylWithDGeom(2.5, 5, fxBody,
                    "Right Intake Wheel" + i, OdeHelper.createCylinder(botSpace, 2.5, 5));
            rightIntakeWheel.setMaterial(intakeMaterial);
            rightIntakeWheel.getTransforms().addAll(new Translate(7.5, (1-i)*6.5, 0), new Rotate(90, Rotate.X_AXIS));
            rightIntakeGroup.getChildren().add(rightIntakeWheel);
        }

        BoxWithDGeom intakeRoof = new BoxWithDGeom(20, 18, 3, fxBody, "intakeRoof");
        intakeRoof.setMaterial(new PhongMaterial(Color.color(0.1, 0, 0, 0)));
        intakeRoof.getTransforms().add(new Translate(0, -0.5, 8 ));

        intakeGroup.getChildren().addAll(leftIntakeGroup, rightIntakeGroup, intakeRoof);
        intakeGroup.getTransforms().add(new Translate(0, -12.5, pltZOffset));

        intakeGroup.updateGeomOffsets();

        botGroup.getChildren().add(intakeGroup);

        zBase = 5.08;

        fxBody.setCategoryBits(CBits.BOT);
        fxBody.setCollideBits(0xFF);
        botBottomMesh.setCategoryBits(CBits.BOT_BOTTOM);
        botBottomMesh.setCollideBits(CBits.FLOOR);
        leftHand.getDGeom().setCategoryBits(CBits.BOT_HANDS | CBits.BOT_LEFT_HAND);
        rightHand.getDGeom().setCategoryBits(CBits.BOT_HANDS | CBits.BOT_RIGHT_HAND);
        for (Node n : leftIntakeGroup.getChildren()) {
            ((CylWithDGeom) n).getDGeom().setCategoryBits(CBits.BOT_INTAKE | CBits.BOT_LEFT_INTAKE);
        }
        for (Node n: rightIntakeGroup.getChildren()) {
            ((CylWithDGeom) n).getDGeom().setCategoryBits(CBits.BOT_INTAKE | CBits.BOT_RIGHT_INTAKE);
        }
        intakeRoof.getDGeom().setCategoryBits(CBits.BOT_INTAKE | CBits.BOT_INTAKE_ROOF);

    }


    @Override
    public synchronized void updateDisplay(){
        super.updateDisplay();
        for (int i=0; i<4; i++) wheelRotates[i].setAngle(wheelRotations[i]);
        leftHandTranslate.setX(handTranslation);
        rightHandTranslate.setX(-handTranslation);
        for (int i=0; i<3; i++) liftTranslates[i].setZ(liftElevation/3.0);
        sliderTranslate.setY(sliderTranslation);
        lift.updateGeomOffsets();
    }


    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        imu.close();
    }

    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup){
        long o1CBits = o1.getCategoryBits();
        long o2CBits = o2.getCategoryBits();

        boolean o1RH = (o1CBits & CBits.BOT_RIGHT_HAND) != 0,
                o2RH = (o2CBits & CBits.BOT_RIGHT_HAND) != 0,
                o1LH = (o1CBits & CBits.BOT_LEFT_HAND) != 0,
                o2LH = (o2CBits & CBits.BOT_LEFT_HAND) != 0,
                o1Block = (o1CBits & CBits.STONES) != 0,
                o2Block = (o2CBits & CBits.STONES) != 0,
                o1RtIntake = (o1CBits & CBits.BOT_RIGHT_INTAKE) != 0,
                o2RtIntake = (o2CBits & CBits.BOT_RIGHT_INTAKE) != 0,
                o1LtIntake = (o1CBits & CBits.BOT_LEFT_INTAKE) != 0,
                o2LtIntake = (o2CBits & CBits.BOT_LEFT_INTAKE) != 0,
                o1BotBottom = (o1CBits & CBits.BOT_BOTTOM) != 0,
                o2BotBottom = (o2CBits & CBits.BOT_BOTTOM) != 0;

        DMatrix3 botRot;
        DVector3 handNorm = new DVector3();

        if (numContacts > 0 && ( o1RH && o2Block || o2RH && o1Block || o1LH && o2Block || o2LH && o1Block)) {
//            System.out.println();
//            System.out.println("o1: " + (o1LH ? "Left Hand" : o1RH ? "Right Hand" : "Block") + "   o2: " + (o2LH ? "Left Hand" : o2RH ? "Right Hand" : "Block"));
            botRot = (DMatrix3)fxBody.getRotation();
            handNorm = new DVector3();
            DMatrix.dMultiply0(handNorm, botRot, o1RH || o2RH? new DVector3(-1, 0, 0) : new DVector3(1, 0, 0));
//            System.out.println("Hand Normal: " + handNorm);
        }


        final double fudgeVerticalLiftSpeed = 0.1;

        for (int i=0; i<numContacts; i++)
        {
            DContact contact = contacts.get(i);
            if ( o1RH && o2Block || o2RH && o1Block || o1LH && o2Block || o2LH && o1Block){
                double dot = handNorm.dot(contact.geom.normal);
//                System.out.println("N: " + i + "  Contact Normal: " + contact.geom.normal + "  Dot: " + dot);

                if (o1Block && dot > 0.8 || o2Block && dot < -0.8) {
                    contact.fdir1.set(0, 0, 1);
                    contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactFDir1 | dContactMotion1 | dContactApprox1_1
                            | dContactApprox1_2 | dContactApprox1_N | dContactRolling | dContactMotion2 | dContactMu2;
                    contact.surface.mu = 6;
                    contact.surface.mu2 = 6;
                    contact.surface.rhoN = 6;
                    contact.surface.soft_cfm = 0.0001;
                    contact.surface.soft_erp = 0.4;
                    if (o1Block) {
                        contact.surface.motion1 = verticalLiftSpeed + fudgeVerticalLiftSpeed;
                        contact.surface.motion2 = o2LH ? -sliderSpeed : sliderSpeed;
                    } else {
                        contact.surface.motion1 = -verticalLiftSpeed + fudgeVerticalLiftSpeed;
                        contact.surface.motion2 = o1LH ? sliderSpeed : -sliderSpeed;
                    }
                } else {
                    contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
                    contact.surface.mu = 0;
                    contact.surface.soft_cfm = 0.00000001;
                    contact.surface.soft_erp = 0.2;
                }


            } else if (o1LtIntake && o2Block || o2LtIntake && o1Block || o1RtIntake && o2Block || o2RtIntake && o1Block){
                contact.fdir1.set(0, 0, 1);
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactFDir1 | dContactMotion1 | dContactApprox1_1
                        | dContactApprox1_2 | dContactApprox1_N | dContactRolling | dContactMotion2 | dContactMu2;
                contact.surface.mu = 0;
                contact.surface.mu2 = 5;
                contact.surface.rhoN = 0;
                contact.surface.soft_cfm = 0.00001;
                contact.surface.soft_erp = 0.2;
                if (o1Block) {
                    contact.surface.motion2 = o2LtIntake? -leftIntakeSpeed : -rightIntakeSpeed;
                } else {
                    contact.surface.motion2 = o1LtIntake? leftIntakeSpeed : rightIntakeSpeed;
                }


            }  else if (o1BotBottom || o2BotBottom){
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
