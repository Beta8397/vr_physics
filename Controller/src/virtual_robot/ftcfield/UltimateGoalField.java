package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import odefx.node_with_geom.BoxWithDGeom;
import odefx.node_with_geom.CylWithDGeom;
import odefx.node_with_geom.GroupWithDGeoms;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.ode4j.ode.OdeConstants.*;

public class UltimateGoalField extends FtcField {

    private static UltimateGoalField ultimateGoalFieldInstance = null;

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    List<FxBody> rings = new ArrayList<>();
    static DMass ringMass = OdeHelper.createMass();

    FxBody[] wobbles = new FxBody[4];

    FxBody[] powershots = new FxBody[3];
    static DMass powershotMass = OdeHelper.createMass();

    private Random rand = new Random();

    public static UltimateGoalField getInstance(Group group, DWorld world, DSpace space){
        if (ultimateGoalFieldInstance == null){
            ultimateGoalFieldInstance = new UltimateGoalField(group, world, space);
        }
        return ultimateGoalFieldInstance;
    }

    public static UltimateGoalField getInstance() {
        if (ultimateGoalFieldInstance == null){
            throw new IllegalStateException("Ultimate Goal Field Instance has not been created yet");
        } else {
            return ultimateGoalFieldInstance;
        }
    }

    private UltimateGoalField(Group group, DWorld world, DSpace space){
        super(group, world, space);
        HALF_FIELD_WIDTH = VirtualRobotController.HALF_FIELD_WIDTH;
    }

    public List<FxBody> getRings(){ return rings; }

    public FxBody getWobble(int i) { return wobbles[i]; }

    @Override
    public void setup() {

        TriangleMesh fieldMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,
                10, 10, false, false, new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return t;
                    }

                    @Override
                    public float z(float s, float t) {
                        return 0;
                    }
                });
        MeshView fieldView = new MeshView(fieldMesh);
        PhongMaterial fieldMaterial = new PhongMaterial();
        fieldMaterial.setDiffuseColor(Color.gray(0.02));
        fieldMaterial.setDiffuseMap(backgroundImage);
        fieldMaterial.setSelfIlluminationMap(backgroundImage);
        fieldView.setMaterial(fieldMaterial);
        subSceneGroup.getChildren().add(fieldView);

        DPlane fieldPlane = OdeHelper.createPlane(space, 0, 0, 1, 0);
        fieldPlane.setData("Field Plane");
        fieldPlane.setCategoryBits(CBits.FLOOR);

        PhongMaterial wallMaterial = new PhongMaterial(Color.color(0.02, 0.02, 0.02, 0));
        for (int i=0; i<4; i++){
            TriangleMesh wallMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,-15, 15,
                    10, 10, false, false, new Util3D.Param3DEqn() {
                        @Override
                        public float x(float s, float t) {
                            return s;
                        }

                        @Override
                        public float y(float s, float t) {
                            return 0;
                        }

                        @Override
                        public float z(float s, float t) {
                            return t;
                        }
                    });
            MeshView wallView = new MeshView(wallMesh);
            wallView.setMaterial(wallMaterial);
            double dx = i==0? 0 : i==1? -HALF_FIELD_WIDTH-2 : i==2? 0 : HALF_FIELD_WIDTH+2;
            double dy = i==0? HALF_FIELD_WIDTH+2 : i==1? 0 : i==2? -HALF_FIELD_WIDTH-2 : 0;
            double angle = i==0? 0 : i==1? 90 : i==2? 180 : -90;
            wallView.getTransforms().addAll(new Translate(dx, dy, 15), new Rotate(angle, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(wallView);
            List<DGeom> wallGeoms = FxBodyHelper.dGeomsFromNode(wallView, space, null);
            for (DGeom dg: wallGeoms) dg.setCategoryBits(CBits.WALLS);
        }

        DMass wobbleMass = OdeHelper.createMass();
        wobbleMass.setCylinderTotal(250, 3, 10, 6.25);

        for (int i=0; i<4; i++) {
            wobbles[i] = FxBody.newInstance(world, space);
            GroupWithDGeoms wobbleGroup = Parts.wobble(i<2? Parts.AllianceColor.BLUE : Parts.AllianceColor.RED, wobbles[i]);
            wobbles[i].setNode(wobbleGroup, false);
            wobbles[i].setMass(wobbleMass);
            subSceneGroup.getChildren().add(wobbleGroup);
            wobbles[i].setDamping(0.05, 0.05);
            wobbles[i].setCategoryBits(CBits.WOBBLES);
        }

        placeWobblesOnField();


        GroupWithDGeoms redGoal = Parts.goalStand(Color.color(1, 0, 0, 0.5), space);
        redGoal.getTransforms().add(new Translate(90, 190, 0));
        redGoal.updateGeomOffsets();
        GroupWithDGeoms blueGoal = Parts.goalStand(Color.color(0, 0, 1, 0.5), space);
        blueGoal.getTransforms().add(new Translate(-90, 190, 0));
        blueGoal.updateGeomOffsets();
        subSceneGroup.getChildren().addAll(redGoal, blueGoal);


        ringMass.setCylinder(1, 3, 6.25, 2);
        ringMass.setMass(100);

        for (int i=0; i<7; i++){
            FxBody ring = FxBody.newInstance(world, space);
            ring.setMass(ringMass);
            MeshView ringView = Parts.ringView();
            ring.setNode(ringView, false);
            DCylinder cyl = OdeHelper.createCylinder(6.25, 2.0);
            cyl.setData("ring");
            ring.addGeom(cyl);
            subSceneGroup.getChildren().add(ringView);
            ring.setCategoryBits(CBits.RINGS);
            ring.setCollideBits(0xFF);
            rings.add(ring);
        }

        placeRingsOnField();

        BoxWithDGeom powershotRail = new BoxWithDGeom(60, 4, 4,space);
        powershotRail.getTransforms().add(new Translate(30, HALF_FIELD_WIDTH + 2, 69));
        powershotRail.setMaterial(new PhongMaterial(Color.SILVER));
        powershotRail.updateGeomOffset();
        subSceneGroup.getChildren().add(powershotRail);

        powershotMass.setCylinder(1, 3, 1.25, 12.5);
        powershotMass.setMass(100);

        for (int i=0; i<3; i++){
            powershots[i] = FxBody.newInstance(world, space);
            powershots[i].setMass(powershotMass);
            CylWithDGeom cyl = new CylWithDGeom(1.25, 12.5, powershots[i]);
            cyl.setMaterial(new PhongMaterial(Color.RED));
            cyl.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
            cyl.updateGeomOffset();
            subSceneGroup.getChildren().add(cyl);
            powershots[i].setNode(cyl, false);
            DMatrix3 rotation = new DMatrix3();
            DRotation.dRFromAxisAndAngle(rotation, 1, 0, 0, Math.toRadians(35));
            DVector3 translation = new DVector3(11.4 + 19.1*i, HALF_FIELD_WIDTH -  6.25 * Math.sin(Math.toRadians(35)),
                    72 +  6.25 * Math.cos(Math.toRadians(35)));
            powershots[i].setPosition(translation);
            powershots[i].setRotation(rotation);
            DHingeJoint hinge = OdeHelper.createHingeJoint(world);
            hinge.attach(powershots[i], null);
            hinge.setAnchor(11.4+19.1*i, HALF_FIELD_WIDTH, 72);
            hinge.setAxis(1, 0, 0);
            hinge.setParamHiStop(0);
            hinge.setParamLoStop(-Math.toRadians(115));
            hinge.setParamBounce(0);
            powershots[i].updateNodeDisplay();
        }

    }

    private void resetPowershotPosition(){
        for (int i=0; i<3; i++){
            DMatrix3 rotation = new DMatrix3();
            DRotation.dRFromAxisAndAngle(rotation, 1, 0, 0, Math.toRadians(35));
            DVector3 translation = new DVector3(11.4 + 19.1*i, HALF_FIELD_WIDTH -  6.25 * Math.sin(Math.toRadians(35)),
                    72 +  6.25 * Math.cos(Math.toRadians(35)));
            powershots[i].setPosition(translation);
            powershots[i].setRotation(rotation);
            powershots[i].updateNodeDisplay();
        }
    }

    private void placeWobblesOnField(){
        for (int i=0; i<4; i++){
            double x = 60 * (1 + i%2);
            if (i<2) x *= -1;
            wobbles[i].setPosition(x, -121, 3);
            DMatrix3 identity = new DMatrix3();
            DRotation.dRSetIdentity(identity);
            wobbles[i].setRotation(identity);
            wobbles[i].updateNodeDisplay();
        }
    }

    private void placeRingsOnField(){
        for (int i=0; i<7; i++){
            if (i<4) {
                rings.get(i).setPosition(92.7, -57.2, 1 + 2 * i);
            } else {
                rings.get(i).setPosition(-92.7, -57.2, 1 + 2 * (i-4));
            }
            DMatrix3 identity = new DMatrix3();
            DRotation.dRSetIdentity(identity);
            rings.get(i).setRotation(identity);
            rings.get(i).getGeom("ring").enable();
            rings.get(i).enable();
            if (!subSceneGroup.getChildren().contains(rings.get(i).getNode())){
                subSceneGroup.getChildren().add(rings.get(i).getNode());
            }
            rings.get(i).updateNodeDisplay();
        }
    }


    @Override
    public void reset() {
        placeRingsOnField();
        placeWobblesOnField();
        resetPowershotPosition();
    }

    @Override
    public void updateDisplay() {
        for (int i=0; i<rings.size(); i++){
            rings.get(i).updateNodeDisplay();
        }
        for (int i=0; i<4; i++) {
            wobbles[i].updateNodeDisplay();
        }
        for (int i=0; i<3; i++){
            powershots[i].updateNodeDisplay();
        }
    }

    @Override
    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        boolean o1Ring = (o1.getCategoryBits() & CBits.RINGS) != 0;
        boolean o2Ring = (o2.getCategoryBits() & CBits.RINGS) != 0;
        boolean bothRings = o1Ring && o2Ring;

        for (int i=0; i<numContacts; i++)
        {
            DContact contact = contacts.get(i);
            if (bothRings) {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;      //Enable bounce
                contact.surface.bounce = 0.1;
                contact.surface.bounce_vel = 4.0;
                contact.surface.mu = 0.3;
                contact.surface.soft_cfm = 0.00002;
                contact.surface.soft_erp = 0.5;
            } else{
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;      //Enable bounce
                contact.surface.bounce = 0.5;
                contact.surface.bounce_vel = 2.0;
                contact.surface.mu = 0.5;
                contact.surface.soft_cfm = 0;
                contact.surface.soft_erp = 0.4;
            }
            DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
            c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }

    @Override
    public void preStepProcess() {
        /*
         * If any ring has left the playing field and is located near floor level (in z) then return it
         * to the field via a "return chute"
         */
        for (int i = 0; i < rings.size(); i++) {
            DVector3 ringPos = (DVector3) rings.get(i).dBodyGetPosition();
            double absX = Math.abs(ringPos.get0());
            double absY = Math.abs(ringPos.get1());
            double z = ringPos.get2();
            if ((z < 10 && (absX > HALF_FIELD_WIDTH || absY > HALF_FIELD_WIDTH)) &&
                    rings.get(i).getGeom("ring").isEnabled()) {
                rings.get(i).setPosition(50, HALF_FIELD_WIDTH, 40);
                double rotationAngle = Math.PI/2 * 1.5 * (0.5 - rand.nextDouble());
                DMatrix3 rotation = new DMatrix3();
                DRotation.dRFromAxisAndAngle(rotation, 0, 1, 0, rotationAngle);
                rings.get(i).setRotation(rotation);
                double linVelDir = Math.PI/6.0 * (0.5 - rand.nextDouble());
                rings.get(i).setLinearVel(100 * Math.sin(linVelDir), -100 * Math.cos(linVelDir), 0);
                rings.get(i).setAngularVel(12.0*Math.sin(rotationAngle), 0, -12*Math.cos(rotationAngle));
            }
        }
    }

}
