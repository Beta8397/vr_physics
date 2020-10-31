package util3d;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.*;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.FxBody;
import odefx.node_with_geom.BoxWithDGeom;
import odefx.node_with_geom.CylWithDGeom;
import odefx.node_with_geom.GroupWithDGeoms;
import odefx.node_with_geom.MeshViewWithDGeom;
import org.ode4j.ode.*;
import virtual_robot.controller.VirtualRobotController;

import java.util.List;

public class Parts {

    public enum AllianceColor {RED, BLUE}

    public static final int BACK_LEFT = 0, FRONT_LEFT = 1, FRONT_RIGHT = 0, BACK_RIGHT = 1;

    public static Group mecanumWheel(double diameter, double width, int type){
        PhongMaterial wheelTreadMaterial = type%2 == 0?
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelA_rotated.jpg") :
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelB_rotated.jpg");
        PhongMaterial wheelSideMaterial = new PhongMaterial(Color.color(0.9, 0.9, 0.9));
        wheelSideMaterial.setSpecularColor(Color.color(0, 0, 0, 0));
        Group wheel = Util3D.cylinder((float)diameter/2.0f, (float)width, 10, 1, 1,
                true, wheelTreadMaterial, wheelSideMaterial);
        return wheel;
    }

    public static Group tetrixBox(float length, float height, float depth, float patternWidth){
        PhongMaterial tetrixMaterial = Util3D.imageMaterial("/virtual_robot/assets/tetrix.jpg");
        return Util3D.patternBox(length, height, depth, patternWidth, patternWidth, patternWidth, tetrixMaterial);
    }

    /**
     *  Create Skystone bridge assembly node
     *  Dimensions now in centimeters.
     * @return
     */
    public static Group skyStoneBridge(DSpace space){
        Group group = new Group();

        double tubeRadius = 1.27;
        double tubeHeight = 127;
        double neutralTubeHeight = 119.38;
        double tubeXOffset = 60.96;
        double tubeYOffset = 7.62;
        double tubeZOffset = 36.83;
        double neutralTubeZOffset = 52.07;
        double bridgeStandThickness = 2.54;
        double bridgeStandWidth = 20.32;
        double innerBridgeStandHeight = 55.88;
        double outerBridgeStandHeight = 40.64;
        double innerBridgeStandXOffset = 58.42;

        PhongMaterial blueBridgeMaterial = new PhongMaterial(Color.BLUE);
        Cylinder blueBridge1 = new Cylinder(tubeRadius, tubeHeight);
        blueBridge1.setMaterial(blueBridgeMaterial);
        blueBridge1.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder blueBridge2 = new Cylinder(tubeRadius, tubeHeight);
        blueBridge2.setMaterial(blueBridgeMaterial);
        blueBridge2.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, -tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom blueBridgeGeom = OdeHelper.createBox(space, tubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        blueBridgeGeom.setPosition(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, 0, tubeZOffset);

        PhongMaterial redBridgeMaterial = new PhongMaterial(Color.RED);
        Cylinder redBridge1 = new Cylinder(tubeRadius, tubeHeight);
        redBridge1.setMaterial(redBridgeMaterial);
        redBridge1.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder redBridge2 = new Cylinder(tubeRadius, tubeHeight);
        redBridge2.setMaterial(redBridgeMaterial);
        redBridge2.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, -tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom redBridgeGeom = OdeHelper.createBox(space, tubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        redBridgeGeom.setPosition(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, 0, tubeZOffset);

        PhongMaterial neutralBridgeMaterial = new PhongMaterial(Color.ORANGE);
        Cylinder neutralBridge1 = new Cylinder(tubeRadius, neutralTubeHeight);
        neutralBridge1.setMaterial(neutralBridgeMaterial);
        neutralBridge1.getTransforms().addAll(
                new Translate(0, tubeYOffset, neutralTubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder neutralBridge2 = new Cylinder(tubeRadius, neutralTubeHeight);
        neutralBridge2.setMaterial(neutralBridgeMaterial);
        neutralBridge2.getTransforms().addAll(
                new Translate(0, -tubeYOffset, neutralTubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom neutralBridgeGeom = OdeHelper.createBox(space, neutralTubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        neutralBridgeGeom.setPosition(0, 0, neutralTubeZOffset);

        PhongMaterial bridgeStandMaterial = new PhongMaterial(Color.CORNSILK);

        Box bridgeStand1 = new Box(bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand1.setMaterial(bridgeStandMaterial);
        bridgeStand1.getTransforms().add(new Translate(-VirtualRobotController.HALF_FIELD_WIDTH-tubeRadius, 0, outerBridgeStandHeight/2.0));

        DGeom bridgeStand1Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand1Geom.setPosition(-VirtualRobotController.HALF_FIELD_WIDTH-tubeRadius, 0, outerBridgeStandHeight/2.0);

        Box bridgeStand2 = new Box(bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand2.setMaterial(bridgeStandMaterial);
        bridgeStand2.getTransforms().add(new Translate(-innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0));

        DGeom bridgeStand2Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand2Geom.setPosition(-innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0);

        Box bridgeStand3 = new Box(bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand3.setMaterial(bridgeStandMaterial);
        bridgeStand3.getTransforms().add(new Translate(innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0));

        DGeom bridgeStand3Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand3Geom.setPosition(innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0);

        Box bridgeStand4 = new Box(bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand4.setMaterial(bridgeStandMaterial);
        bridgeStand4.getTransforms().add(new Translate(VirtualRobotController.HALF_FIELD_WIDTH+tubeRadius, 0, outerBridgeStandHeight/2.0));

        DGeom bridgeStand4Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand4Geom.setPosition(VirtualRobotController.HALF_FIELD_WIDTH+tubeRadius, 0, outerBridgeStandHeight/2.0);

        group.getChildren().addAll(blueBridge1, blueBridge2, redBridge1, redBridge2, neutralBridge1, neutralBridge2,
                bridgeStand1, bridgeStand2, bridgeStand3, bridgeStand4);

        return group;
    }

    /**
     * Create a MeshView object that represents a ring for the Ultimate Goal competition
     * @return
     */
    public static MeshView ringView(){
        PhongMaterial orangeMat = new PhongMaterial(Color.ORANGE);
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
        return ringView;
    }


    public static GroupWithDGeoms goalStand(Color color, DSpace space){
        PhongMaterial mat = new PhongMaterial(color);
//        BoxWithDGeom bottomBox = new BoxWithDGeom(60, 15, 32, space);
//        bottomBox.getTransforms().add(new Translate(0, 0, 16));
        Box bottomBox = new Box(60, 15, 30);
        bottomBox.getTransforms().add(new Translate(0, 0, 15));
        BoxWithDGeom spacer = new BoxWithDGeom(60, 15, 2, space);
        spacer.getTransforms().add(new Translate(0, 0, 31));
        BoxWithDGeom leftVerticalBox = new BoxWithDGeom(4, 15, 50, space);
        leftVerticalBox.getTransforms().add(new Translate(-28, 0, 57));
        BoxWithDGeom rightVerticalBox = new BoxWithDGeom(4, 15, 50, space);
        rightVerticalBox.getTransforms().add(new Translate(28, 0, 57));
        BoxWithDGeom leftTopBox = new BoxWithDGeom(10, 15, 14, space);
        leftTopBox.getTransforms().add(new Translate(-25, 0, 89));
        BoxWithDGeom rightTopBox = new BoxWithDGeom(10, 15, 14, space);
        rightTopBox.getTransforms().add(new Translate(25, 0, 89));
        BoxWithDGeom topBox = new BoxWithDGeom(60, 15, 30, space);
        topBox.getTransforms().add(new Translate(0, 0, 111));
        BoxWithDGeom midHorizontalBox = new BoxWithDGeom(52, 15, 2, space);
        midHorizontalBox.getTransforms().add(new Translate(0, 0, 52));
        BoxWithDGeom topHorizontalBox = new BoxWithDGeom(52, 15, 2.5, space);
        topHorizontalBox.getTransforms().add(new Translate(0, 0, 84));
        GroupWithDGeoms group = new GroupWithDGeoms();
        group.getChildren().addAll(bottomBox, spacer, leftVerticalBox, rightVerticalBox, leftTopBox, rightTopBox,
                topBox, midHorizontalBox, topHorizontalBox);
        for (Node n: group.getChildren()){
            ((Shape3D)n).setMaterial(mat);
        }
        return group;
    }

    public static GroupWithDGeoms wobble(AllianceColor allianceColor, FxBody fb){
        final float R = 11.333f;
        final double tMax = Math.toRadians(62);
        TriangleMesh wobbleBaseMesh = Util3D.getParametricMesh(0, 2 * Math.PI, 0, tMax, 36, 18, true, false,
                new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return R * (float)Math.cos(s) * (float)Math.sin(t);
                    }

                    @Override
                    public float y(float s, float t) {
                        return R * (float)Math.sin(s) * (float)Math.sin(t);
                    }

                    @Override
                    public float z(float s, float t) {
                        return -R * (float)Math.cos(t);
                    }
                });
        MeshView wobbleBaseMeshView = new MeshViewWithDGeom(wobbleBaseMesh, fb);
        wobbleBaseMeshView.setMaterial(new PhongMaterial(Color.BLACK));
        CylWithDGeom diskView = new CylWithDGeom(10, 2, fb);
        diskView.getTransforms().addAll(new Translate(0, 0, -4.33), new Rotate(90, Rotate.X_AXIS));
        diskView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.RED : Color.BLUE));
        CylWithDGeom handleView = new CylWithDGeom(1.25, 18, fb);
        handleView.getTransforms().addAll(new Translate(0, 0, 5.67), new Rotate(90, Rotate.X_AXIS));
        handleView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.DARKRED : Color.DARKBLUE));
        CylWithDGeom topView = new CylWithDGeom(2, 5, fb);
        topView.setDGeom(OdeHelper.createCylinder(3, 5));
        topView.getTransforms().addAll(new Translate(0, 0, 17.17), new Rotate(90, Rotate.X_AXIS));
        topView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.DARKRED : Color.DARKBLUE));
        GroupWithDGeoms wobbleGroup = new GroupWithDGeoms();
        wobbleGroup.getChildren().addAll(wobbleBaseMeshView, diskView, handleView, topView);
        wobbleGroup.getTransforms().add(new Translate(0, 0, 8.33));
        wobbleGroup.updateGeomOffsets();
        return wobbleGroup;
    }

    public static Group wobble(AllianceColor allianceColor){
        final float R = 11.333f;
        final double tMax = Math.toRadians(62);
        TriangleMesh wobbleBaseMesh = Util3D.getParametricMesh(0, 2 * Math.PI, 0, tMax, 36, 18, true, false,
                new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return R * (float)Math.cos(s) * (float)Math.sin(t);
                    }

                    @Override
                    public float y(float s, float t) {
                        return R * (float)Math.sin(s) * (float)Math.sin(t);
                    }

                    @Override
                    public float z(float s, float t) {
                        return -R * (float)Math.cos(t);
                    }
                });
        MeshView wobbleBaseMeshView = new MeshView(wobbleBaseMesh);
        wobbleBaseMeshView.setMaterial(new PhongMaterial(Color.BLACK));
        Cylinder diskView = new Cylinder(10, 2);
        diskView.getTransforms().addAll(new Translate(0, 0, -4.33), new Rotate(90, Rotate.X_AXIS));
        diskView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.RED : Color.BLUE));
        Cylinder handleView = new Cylinder(1.25, 18);
        handleView.getTransforms().addAll(new Translate(0, 0, 5.67), new Rotate(90, Rotate.X_AXIS));
        handleView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.DARKRED : Color.DARKBLUE));
        Cylinder topView = new Cylinder(2, 5);
        topView.getTransforms().addAll(new Translate(0, 0, 17.17), new Rotate(90, Rotate.X_AXIS));
        topView.setMaterial(new PhongMaterial(allianceColor == AllianceColor.RED? Color.DARKRED : Color.DARKBLUE));
        Group wobbleGroup = new Group();
        wobbleGroup.getChildren().addAll(wobbleBaseMeshView, diskView, handleView, topView);
        wobbleGroup.getTransforms().add(new Translate(0, 0, 8.33));
        return wobbleGroup;
    }


}
