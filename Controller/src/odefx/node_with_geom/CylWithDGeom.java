package odefx.node_with_geom;


import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import odefx.FxBody;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeHelper;

public class CylWithDGeom extends Cylinder implements Shape3DWithDGeom {
    private String name = "CylWithDGeom";
    private DGeom dGeom;
    private Rotate relativeGeomRotate = null;
    private Translate relativeGeomTranslate = null;


    public CylWithDGeom(double radius, double height, FxBody fb){
        super(radius, height);
        dGeom = OdeHelper.createCylinder(radius, height);
        fb.addGeom(dGeom);
    }

    public CylWithDGeom(double radius, double height, FxBody fb, String name) {
        this(radius, height, fb);
        this.name = name;
        this.dGeom.setData(name);
    }

    public CylWithDGeom(double radius, double height, FxBody fb, DGeom geom){
        super(radius, height);
        dGeom = geom;
        fb.addGeom(dGeom);
    }

    public CylWithDGeom(double radius, double height, FxBody fb, String name, DGeom geom){
        super(radius, height);
        dGeom = geom;
        fb.addGeom(dGeom);
        this.name = name;
    }

    public void setDGeom(DGeom geom){
        DBody db = dGeom == null? null : dGeom.getBody();
        if (dGeom != null) dGeom.destroy();
        dGeom = geom;
        if (dGeom != null && db != null) dGeom.setBody(db);
    }
    public DGeom getDGeom(){ return dGeom; }

    public void setName(String name){
        this.name = name;
        if (dGeom != null) dGeom.setData(name);
    }

    public String getName(){ return name; }

    public void setRelGeomRotate(Rotate rotate){ this.relativeGeomRotate = rotate; }

    public Rotate getRelGeomRotate(){ return relativeGeomRotate; }

    public void setRelGeomOffset(Translate translate) { this.relativeGeomTranslate = translate; }

    public Translate getRelGeomOffset () { return relativeGeomTranslate; }

    public void updateGeomOffset(Transform preTransform){
        if (dGeom == null) return;
        Transform transform = preTransform.clone();
        ObservableList<Transform> nodeTransforms = this.getTransforms();
        for (int i=0; i<nodeTransforms.size(); i++) transform = transform.createConcatenation(nodeTransforms.get(i));
        Rotate nodeRelGeomRotate = this.getRelGeomRotate();
        Translate nodeRelGeomOffset = this.getRelGeomOffset();
        if (nodeRelGeomOffset != null) transform = transform.createConcatenation(nodeRelGeomOffset);
        if (nodeRelGeomRotate != null) transform = transform.createConcatenation(nodeRelGeomRotate);
        transform = transform.createConcatenation(new Rotate(90, new Point3D(1, 0, 0)));
        double[] tData = transform.toArray(MatrixType.MT_3D_3x4);
        if (dGeom.getBody() != null) {
            dGeom.setOffsetPosition(tData[3], tData[7], tData[11]);
            DMatrix3 dRotMatrix = new DMatrix3(tData[0], tData[1], tData[2], tData[4], tData[5], tData[6],
                    tData[8], tData[9], tData[10]);
            dGeom.setOffsetRotation(dRotMatrix);
        } else {
            dGeom.setPosition(tData[3], tData[7], tData[11]);
            DMatrix3 dRotMatrix = new DMatrix3(tData[0], tData[1], tData[2], tData[4], tData[5], tData[6],
                    tData[8], tData[9], tData[10]);
            dGeom.setRotation(dRotMatrix);
        }
    }


}
