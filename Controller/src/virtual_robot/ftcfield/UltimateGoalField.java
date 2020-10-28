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
import odefx.node_with_geom.GroupWithDGeoms;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import java.util.ArrayList;
import java.util.List;

import static org.ode4j.ode.OdeConstants.*;

public class UltimateGoalField extends FtcField {

    private static UltimateGoalField ultimateGoalFieldInstance = null;

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    List<FxBody> rings = new ArrayList<>();
    static DMass ringMass = OdeHelper.createMass();

    FxBody[] wobbles = new FxBody[4];

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
        wobbleMass.setCylinderTotal(1000, 3, 10, 6.25);

        for (int i=0; i<4; i++) {
            Group wobbleGroup = Parts.wobble(i<2? Parts.AllianceColor.BLUE : Parts.AllianceColor.RED);
            wobbles[i] = FxBody.newInstance(world, space);
            wobbles[i].setNode(wobbleGroup, true);
            wobbles[i].setMass(wobbleMass);
            double x = 60 * (1 + i%2);
            if (i<2) x *= -1;
            wobbles[i].setPosition(x, -120, 3);
            subSceneGroup.getChildren().add(wobbleGroup);
            wobbles[i].setDamping(0.05, 0.05);
            wobbles[i].updateNodeDisplay();
            wobbles[i].setCategoryBits(CBits.WOBBLES);
        }


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
            if (i<4){
                //Initial Rings on the field
                ring.setPosition(92.7, -57.2, 1 + 2*i);
                subSceneGroup.getChildren().add(ringView);
            } else {
                //Initial Rings on the bot
                ring.getGeom("ring").disable();
                ring.disable();
            }
            ring.setCategoryBits(CBits.RINGS);
            ring.setCollideBits(0xFF);
            rings.add(ring);
        }
    }


    @Override
    public void reset() {
        for (int i=0; i<rings.size(); i++){
            FxBody ring = rings.get(i);
            if (i<4){
                //Initial Rings on the field
                ring.setPosition(92.7, -57.2, 1 + 2*i);
                DMatrix3 rot = new DMatrix3();
                DRotation.dRFromAxisAndAngle(rot, 0, 0, 1, 0);
                ring.setRotation(rot);
                if (!subSceneGroup.getChildren().contains(ring.getNode())) subSceneGroup.getChildren().add(ring.getNode());
                ring.getGeom("ring").enable();
                ring.enable();
            } else {
                //Initial Rings on the bot
                if (subSceneGroup.getChildren().contains(ring.getNode())) subSceneGroup.getChildren().remove(ring.getNode());
                ring.getGeom("ring").disable();
                ring.disable();
                ring.setPosition(0,0,0);
            }
        }
    }

    @Override
    public void updateDisplay() {
        for (int i=0; i<rings.size(); i++){
            rings.get(i).updateNodeDisplay();
        }
        for (int i=0; i<4; i++) {
            wobbles[i].updateNodeDisplay();
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
//        for (FxBody2 s: rings){
//            DVector3C linVel = s.getLinearVel();
//            DVector3C angVel = s.getAngularVel();
//            s.addForce(linVel.reScale(-0.1*s.getMass().getMass()));
//            s.addTorque(angVel.reScale(-0.1*s.getMass().getI().get22()));
//        }
    }

}
