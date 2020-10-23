package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBody2;
import odefx.FxBodyHelper;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import util3d.ClosedMeshCreator;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import javax.swing.text.View;
import java.util.List;

import static org.ode4j.ode.OdeConstants.*;

public class UltimateGoalField extends FtcField {

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    FxBody2[] rings = new FxBody2[7];
    DMass ringMass = OdeHelper.createMass();
    PhongMaterial orangeMat = new PhongMaterial(Color.ORANGE);

    public UltimateGoalField(Group group, DWorld world, DSpace space){
        super(group, world, space);
        HALF_FIELD_WIDTH = VirtualRobotController.HALF_FIELD_WIDTH;
    }

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

//        DSpace bridgeSpace = OdeHelper.createSimpleSpace(space);
//        Group bridgeGroup = Parts.skyStoneBridge(bridgeSpace);
//        for (DGeom g: bridgeSpace.getGeoms()){
//            g.setCategoryBits(CBits.BRIDGE);
//        }
//
//
//        subSceneGroup.getChildren().add(bridgeGroup);

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




        ringMass.setCylinder(1, 3, 6.25, 2);
        ringMass.setMass(100);

        for (int i=0; i<7; i++){
            rings[i] = FxBody2.newInstance(world, space);
            rings[i].setMass(ringMass);
            Mesh ringMesh = Util3D.getParametricMesh(0, 2 * Math.PI, 0, -2 * Math.PI, 20, 10, true, true, new Util3D.Param3DEqn() {
                @Override
                public float x(float s, float t) {
                    return (float)Math.cos(s) * (5 + 1.25f * (float)Math.cos(t));
                }

                @Override
                public float y(float s, float t) {
                    return (float)Math.sin(s) * (5 + 1.25f * (float)Math.cos(t));
                }

                @Override
                public float z(float s, float t) {
                    return (float) (float)Math.sin(t);
                }
            });
            MeshView ringView = new MeshView(ringMesh);
            ringView.setMaterial(orangeMat);
            subSceneGroup.getChildren().add(ringView);
            rings[i].setNode(ringView, true);
            rings[i].setPosition(100*Math.cos(i*2*Math.PI/7), 100*Math.sin(i*2*Math.PI/7), 1);
            rings[i].setCategoryBits(CBits.STONES);
            rings[i].setCollideBits(0xFF);
        }
    }

    @Override
    public void reset() {
        for (int i=0; i<rings.length; i++){
            rings[i].setPosition(100*Math.cos(i*2*Math.PI/7), 100*Math.sin(i*2*Math.PI/7), 1);
        }
    }

    @Override
    public void updateDisplay() {
        for (int i=0; i<rings.length; i++){
            rings[i].updateNodeDisplay();
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
        for (FxBody2 s: rings){
            DVector3C linVel = s.getLinearVel();
            DVector3C angVel = s.getAngularVel();
            s.addForce(linVel.reScale(-0.1*s.getMass().getMass()));
            s.addTorque(angVel.reScale(-0.1*s.getMass().getI().get22()));
        }
    }

}
