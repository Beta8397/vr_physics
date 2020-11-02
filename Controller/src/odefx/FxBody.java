package odefx;

import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.Node;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.math.*;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.Objects_H;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import static org.ode4j.ode.internal.Common.dAASSERT;

public class FxBody extends DxBody {

    private Node node = null;

    private DSpace dSpace = null;

    private HashMap<String, DGeom> geoms = new HashMap<>();

    /**
     * The purpose of children is to allow a group of bodies to be repositioned together, maintaining their
     * original spatial relationships. The only methods that use the children of the FxBody are
     * getChildren(), setPosition(), setRotation(), setQuaternion(), and updateNodeDisplay().
     */
    private List<FxBody> children = new ArrayList<>();

    private FxBody(DxWorld world){
        super(world);
    }

    public static FxBody newInstance(DWorld world){

        DxWorld w = (DxWorld)world;
        dAASSERT (w);
        FxBody b = new FxBody(w);
        b.firstjoint.set(null);
        b.flags = 0;
        b.geom = null;
        b.average_lvel_buffer = null;
        b.average_avel_buffer = null;
        //TZ
        b.mass = new DxMass();
        b.mass.dMassSetParameters (1, 0,0,0,1,1,1,0,0,0);
        b.invI = new DMatrix3();
        //MAT.dSetZero (b.invI.v,4*3);
        b.invI.set00( 1 );
        b.invI.set11( 1 );
        b.invI.set22( 1 );
        b.invMass = 1;
        b._posr = new Objects_H.DxPosR();
        //MAT.dSetZero (b.posr.pos.v,4);
        b._q = new DQuaternion();
        //MAT.dSetZero (b._q.v,4);
        b._q.set( 0, 1 );
        b._posr.Rw().setIdentity();
        b.lvel = new DVector3();
        //MAT.dSetZero (b.lvel.v,4);
        b.avel = new DVector3();
        //MAT.dSetZero (b.avel.v,4);
        b.facc = new DVector3();
        //MAT.dSetZero (b.facc.v,4);
        b.tacc = new DVector3();
        //MAT.dSetZero (b.tacc.v,4);
        b.finite_rot_axis = new DVector3();
        //MAT.dSetZero (b.finite_rot_axis.v,4);
        //addObjectToList (b,(dObject **) &w.firstbody);
        addObjectToList(b, w.firstbody);
        w.nb++;

        // set auto-disable parameters
        b.average_avel_buffer = b.average_lvel_buffer = null; // no buffer at beginning
        b.dBodySetAutoDisableDefaults ();	// must do this after adding to world
        b.adis_stepsleft = b.adis.idle_steps;
        b.adis_timeleft = b.adis.idle_time;
        b.average_counter = 0;
        b.average_ready = 0; // average buffer not filled on the beginning
        b.dBodySetAutoDisableAverageSamplesCount(b.adis.average_samples);

        b.moved_callback = null;

        b.dBodySetDampingDefaults();	// must do this after adding to world

        b.flags |= w.body_flags & dxBodyMaxAngularSpeed;
        b.max_angular_speed = w.max_angular_speed;

        b.flags |= dxBodyGyroscopic;

        return b;
    }

    /**
     * Create new instance of FxBody2 with new DBody
     * @param world DBody will be placed in this world
     * @param space DGeoms created/added to this FxBody will be placed in this botSpace
     * @return
     */
    public static FxBody newInstance(DWorld world, DSpace space){
        FxBody fxBody2 = newInstance(world);
        fxBody2.dSpace = space;
        return fxBody2;
    };

    /**
     * Get the JavaFx Node (should be either a Group or Shape3D) associated with this FxBody
     * @return
     */
    public Node getNode() {
        return node;
    }

    /**
     * Get the DSpace associated with this FxBody.
     * @return
     */
    public DSpace getSpace() {
        return dSpace;
    }

    /**
     * Set the DSpace associated with this FxBody, and add all DGeoms to the DSpace
     * @param dSpace
     */
    public void setSpace(DSpace dSpace) {
        this.dSpace = dSpace;
        DGeom dGeom = this.getFirstGeom();
        while (dGeom != null){
            if (dSpace == null) {
                DSpace s = dGeom.getSpace();
                if (s != null) s.remove(dGeom);
            } else {
                dSpace.add(dGeom);
            }
            dGeom = this.getNextGeom(dGeom);
        }
    }

    public List<FxBody> getChildren() { return children; }

    /**
     * Add a DGeom to the DBody, and also to the botSpace
     * @param dGeom
     */
    public void addGeom(DGeom dGeom){
        dGeom.setBody(this);
        if (dSpace != null && dGeom.getSpace() == null) dSpace.add(dGeom);
        if (dGeom.getData() != null && dGeom.getData() instanceof String) geoms.put((String)dGeom.getData(), dGeom);
    }

    public void addGeom(DGeom dGeom, double x, double y, double z){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
    }

    public Iterable<DGeom> getGeoms(){
        return new Iterable<DGeom>() {
            @Override
            public Iterator<DGeom> iterator() {
                return new Iterator<DGeom>() {
                    DGeom current = null;

                    @Override
                    public boolean hasNext() {
                        if (current == null) return getFirstGeom() != null;
                        else return getNextGeom(current) != null;
                    }

                    @Override
                    public DGeom next() {
                        current = current == null? getFirstGeom() : getNextGeom(current);
                        return current;
                    }
                };
            }
        };
    }

    /**
     * Add a DGeom to the DBody, and to the botSpace, applying the requested offset position and rotation
     * @param dGeom
     * @param x   x offset
     * @param y   y offset
     * @param z   z offset
     * @param R   Rotation offset
     */
    public void addGeom(DGeom dGeom, double x, double y, double z, DMatrix3C R){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
        dGeom.setOffsetRotation(R);
    }

    public void addGeom(DGeom dGeom, double x, double y, double z, DQuaternion q){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
        dGeom.setOffsetQuaternion(q);
    }

    /**
     * Return dGeom whose "key" is name. Note not all geoms will necessarily have a key.
     * @param name
     * @return
     */
    public DGeom getGeom(String name){
        return geoms.get(name);
    }


    /**
     * Set category bits to the same value for all geoms belonging to this FxBody
     * @param bits
     */
    public void setCategoryBits(long bits){
        for (DGeom g: getGeoms()){
            g.setCategoryBits(bits);
        }
    }

    public void destroy(boolean destroyChildren){
        //First, destroy all of the dgeoms
        List<DGeom> dGeoms = new ArrayList<>();
        for (DGeom g: getGeoms()) dGeoms.add(g);
        for (int i=0; i<dGeoms.size(); i++){
            DGeom g = dGeoms.get(i);
            if (g.getSpace() != null) g.getSpace().remove(g);
            g.destroy();
        }

        //Next, destroy all of the joints
        List<DJoint> joints = new ArrayList<>();
        int numJoints = getNumJoints();
        for (int i=0; i<numJoints; i++){
            joints.add(getJoint(i));
        }
        while (joints.size() > 0){
            DJoint j = joints.get(0);
            joints.remove(0);
            j.destroy();
        }

        //Next, destroy all of the children
        if (children != null){
            for (FxBody child: children){
                child.destroy(true);
            }
        }

        //Finally, destroy the DBody
        super.destroy();
    }

    @Override
    public void destroy(){
        destroy(true);
    }

    /**
     * Set collide bits to the same value for all geoms belonging to this FxBody
     * @param bits
     */
    public void setCollideBits(long bits){
        for (DGeom g: getGeoms()){
            g.setCollideBits(bits);
        }
    }


    /**
     * Set category bits for a single geom. If there is no geom with the provided ID, an exception will result
     * @param geomId
     * @param bits
     */
    public void setCategoryBits(String geomId, long bits){
        geoms.get(geomId).setCategoryBits(bits);
    }

    /**
     * Set collide bits for a single geom. If there is no geom with the provided ID, an exception will result
     * @param geomId
     * @param bits
     */
    public void setCollideBits(String geomId, long bits){
        geoms.get(geomId).setCollideBits(bits);
    }


    /**
     * Set the JavaFX node associated with this FxBody and optionally generate a list of DGeoms corresponding to
     * the JavaFX node, and add them to the DBody associated with this FxBody. If the node has transforms,
     * they will be used to generate appropriate Offsets for the generated DGeoms. setNode() will automatically
     * add a Translate transform and a Rotate transform to the beginning of the Transforms property of the
     * node. Those transforms will be used to update the display as the dBody object moves.
     *
     * @param node  The JavaFX Node Object -- should be Group or Shape3D.
     * @param generateGeoms If true, generate DGeom objects and associate them with the DBody.
     */
    public void setNode(Node node, boolean generateGeoms) {
        if (generateGeoms) {
            FxBodyHelper.dGeomsFromNode(node, dSpace, this);
        }
        this.node = node;
        this.node.getTransforms().add(0, new Rotate(0));
        this.node.getTransforms().add(0, new Translate(0,0,0));
    }

    /**
     * Update the display of the Node, based on current position and orientation of the DBody.
     *
     * This method should only be called from the Application Thread. This can be accomplished by
     * wrapping the call in a call to Platform.runLater.
     */
    public void updateNodeDisplay(boolean updateChildren){
        if (node == null) return;
        ObservableList<Transform> transforms = node.getTransforms();
        if (transforms.size() < 2) return;
        DVector3C pos = getPosition();
        DQuaternionC quat = getQuaternion();
        DVector3 axis = new DVector3(quat.get(1), quat.get(2), quat.get(3));
        double sinThetaOver2 = axis.length();
        double angle = 2.0 * Math.atan2(sinThetaOver2, quat.get(0)) * 180.0 / Math.PI;
        //Normalize the axes[i] to a length of 1 (but not if the angle is zero). If the angle
        //is zero, the axes[i] length will also be zero and this would crash. May not need to
        //normalize at all.
        //if (sinThetaOver2 != 0) axis.normalize();
        ((Rotate)transforms.get(1)).setAxis(new Point3D(axis.get0(), axis.get1(), axis.get2()));
        ((Rotate)transforms.get(1)).setAngle(angle);
        ((Translate)transforms.get(0)).setX(pos.get0());
        ((Translate)transforms.get(0)).setY(pos.get1());
        ((Translate)transforms.get(0)).setZ(pos.get2());

        if (updateChildren){
            for (FxBody fbChild: children){
                fbChild.updateNodeDisplay(true);
            }
        }
    }

    public void updateNodeDisplay(){
        updateNodeDisplay(false);
    }

    /**
     * Set position of the DBody, and automatically update display. Call only from the Application thread.
     * @param x
     * @param y
     * @param z
     */
    public void setPosition(double x, double y, double z, boolean setChildPos) {
        DVector3C oldPos = getPosition().clone();
        super.setPosition(x, y, z);
        if (setChildPos) {
            for (FxBody child : children) {
                child.setPosition(((DVector3)child.getPosition()).reAdd(getPosition()).reSub(oldPos), true);
            }
        }
        updateNodeDisplay();
    }

    public void setPosition(double x, double y, double z) { setPosition(x, y, z, true); }


    /**
     * Set the position of the DBody, and automatically update display. Call only from the application thread.
     * @param p
     */
    public void setPosition(DVector3C p, boolean setChildPos) {
        DVector3C oldPos = getPosition().clone();
        super.setPosition(p);
        if (setChildPos) {
            for (FxBody child : children) {
                child.setPosition(((DVector3)child.getPosition()).reAdd(getPosition()).reSub(oldPos), true);
            }
        }
        updateNodeDisplay();
    }

    public void setPosition(DVector3C p) { setPosition(p, true); }


    /**
     * Set the Rotation of the DBody, and automatically update display. Call only from the application thread.
     * @param R
     */
    public void setRotation(DMatrix3C R, boolean setChildRot) {
        DMatrix3C oldRot = getRotation().clone();
        super.setRotation(R);
        if (setChildRot){
            DMatrix3 invOldRot = new DMatrix3();
            DMatrix.dInvertPDMatrix(oldRot, invOldRot);
            for (FxBody child: children){
                DMatrix3 newRotTimesInvOldRot = new DMatrix3();
                DMatrix.dMultiply0(newRotTimesInvOldRot, R, invOldRot);
                DMatrix3 newChildRot = new DMatrix3();
                DMatrix.dMultiply0(newChildRot, newRotTimesInvOldRot, child.getRotation());
                DVector3 newChildPos = new DVector3();
                DMatrix.dMultiply0(newChildPos, newRotTimesInvOldRot, child.getPosition().reSub(getPosition()));
                newChildPos.add(getPosition());
                child.setPosition(newChildPos, true);
                child.setRotation(newChildRot, true);
            }
        }
        updateNodeDisplay();
    }

    public void setRotation(DMatrix3C R) { setRotation(R, true);}


    /**
     * Set the Quaternion of the DBody, and automatically update display. Call only from the application thread.
     * @param q
     */
    public void setQuaternion(DQuaternionC q, boolean setChildRot) {
        DMatrix3C oldRot = getRotation().clone();
        super.setQuaternion(q);
        if (setChildRot){
            DMatrix3 invOldRot = new DMatrix3();
            DMatrix.dInvertPDMatrix(oldRot, invOldRot);
            for (FxBody child: children){
                DMatrix3 newRot = new DMatrix3();
                DRotation.dRfromQ(newRot, q);
                DMatrix3 RotNewTimesInvOldRot = new DMatrix3();
                DMatrix.dMultiply0(RotNewTimesInvOldRot, newRot, invOldRot);
                DMatrix3 newChildRot = new DMatrix3();
                DMatrix.dMultiply0(newChildRot, RotNewTimesInvOldRot, child.getRotation());
                DVector3 newChildPos = new DVector3();
                DMatrix.dMultiply0(newChildPos, RotNewTimesInvOldRot, child.getPosition().reSub(getPosition()));
                newChildPos.add(getPosition());
                child.setPosition(newChildPos, true);
                child.setRotation(newChildRot, true);
            }
        }
        updateNodeDisplay();
    }

    public void setQuaternion(DQuaternionC q) { setQuaternion(q, true);}

}
