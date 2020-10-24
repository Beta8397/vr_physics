package odefx.node_with_geom;

import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DGeom;

public class GroupWithDGeoms extends Group {

    public GroupWithDGeoms(){
        super();
    }

    public GroupWithDGeoms(String id){
        super();
        this.setId(id);
    }

    public void updateGeomOffsets(Transform preTransform){

        Transform transform = preTransform == null? new Translate(0, 0, 0) : preTransform.clone();
        ObservableList<Transform> transforms = this.getTransforms();
        for (int i=0; i<transforms.size(); i++) transform = transform.createConcatenation(transforms.get(i));

        for (Node node: getChildren()){
            if ( !(node instanceof GroupWithDGeoms) && !(node instanceof Shape3DWithDGeom)) continue;

            if (node instanceof GroupWithDGeoms){
                ((GroupWithDGeoms)node).updateGeomOffsets(transform);
            } else {
                ((Shape3DWithDGeom)node).updateGeomOffset(transform);
            }

        }
    }


    public void updateGeomOffsets(){
        updateGeomOffsets(null);
    }

}
