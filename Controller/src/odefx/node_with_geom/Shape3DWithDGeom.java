package odefx.node_with_geom;

import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.ode.DGeom;

/**
 * Represents a simple JavaFX Shape3D (Box, Cylinder, Sphere) with an attached ODE4J DGeom. The DGeom would
 * typically correspond in size, shape, and position to the JavaFX Shape3D, but this is not mandatory.
 */
public interface Shape3DWithDGeom {

    void setName(String name);
    String getName();

    /**
     * Set offset of the DGeom relative to the Shape3D
     * @param translate
     */
    void setRelGeomOffset(Translate translate);

    /**
     * Get offset of the DGeom relative to the Shape3D
     * @return
     */
    Translate getRelGeomOffset();

    /**
     * Set rotation of the DGeom relative to the Shape3D
     * @param rotate
     */
    void setRelGeomRotate(Rotate rotate);

    /**
     * Get rotation of the DGeom relative to the Shape3D
     * @return
     */
    Rotate getRelGeomRotate();

    /**
     * Set the DGeom
     * @param dGeom
     */
    void setDGeom(DGeom dGeom);

    /**
     * Get the DGeom
     * @return
     */
    DGeom getDGeom();

    /**
     * Update the Total offset of the DGeom within its FxBody. This will correspond to transform matrix of
     * the Shape3D, with additional application of the specified pre-transform. The pre-transform would typically
     * be the cumulative transform needed because the Shape3D is nested within one or more Group objects that have
     * their own transforms.
     * @param preTransform
     */
    void updateGeomOffset(Transform preTransform);

    /**
     * Update the Total offset of the DGeom within its FxBody. This no-parameter method would be used if the
     * Shape3D is not nested within any Group(s).
     */
    default void updateGeomOffset(){updateGeomOffset(new Translate(0,0,0));}

}
