package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.DepreciatedPlane3f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.MathEx;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;

/**
 * This class simulates the viewable space of a camera
 */
public class FrustumInterpolator {

    private DepreciatedMatrix4f imgToLocal; //camera matrix will be used to modify frustum coordinates in local space
    private DepreciatedMatrix4f camRot;

    private double hFOV, vFOV; //horizontal and vertical fov, important for calculating the frustum later
    private DepreciatedVector4F fplane_right = new DepreciatedVector4F(),
                     fplane_bottom = new DepreciatedVector4F(),
                     fplane_center = new DepreciatedVector4F();

    private DepreciatedVector3F camPos = new DepreciatedVector3F();
    private DepreciatedPlane3f cardinalAxisPlane = DepreciatedPlane3f.XZ_PLANE;

    //presets for listed cameras
    public static FrustumInterpolator Logitech_C270 = new FrustumInterpolator(MathEx.findFOV(3.58, 2.02, 4.11));

    public FrustumInterpolator(double horizontal_fov, double vertical_fov, DepreciatedMatrix4f cam_rot, DepreciatedVector3F cam_pos){
        this.hFOV = horizontal_fov;
        this.vFOV = vertical_fov;

        this.imgToLocal = new DepreciatedMatrix4f();
        this.camRot = cam_rot;
        this.camPos = cam_pos;

        this.setupFrustum();
    }

    public FrustumInterpolator(double[] fov, DepreciatedMatrix4f cam_rot, DepreciatedVector3F cam_pos){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.imgToLocal = new DepreciatedMatrix4f();
        this.camRot = cam_rot;
        this.camPos = cam_pos;

        this.setupFrustum();
    }

    public FrustumInterpolator(double[] fov){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.imgToLocal = new DepreciatedMatrix4f();
        this.camRot = new DepreciatedMatrix4f(); //cam_rot;
        this.camPos = new DepreciatedVector3F(); //cam_pos;

        this.setupFrustum();
    }

    /**
     * Usually a method called in the constructor and setters to update the matrices used
     * <br> However, this method is available because this class doesn't work with MOMM
     */

    public float fpr_, fpb_;
    public void setupFrustum() {
        double angV = vFOV * 0.5;
        double angH = hFOV * 0.5;

        float fpDistance = 1000;

        float fpr = (float) (Math.tan(angH) * fpDistance);
        float fpb = (float) (Math.tan(angV) * -1 * fpDistance);

        this.fpr_ = fpr; this.fpb_ = fpb;

        fplane_right = camRot.matMul(new DepreciatedVector4F(fpr, 0, 0, 1));
        fplane_bottom = camRot.matMul(new DepreciatedVector4F(0, fpb, 0, 1));
        fplane_center = camRot.matMul(new DepreciatedVector4F(0, 0, fpDistance, 1));

        imgToLocal = new DepreciatedMatrix4f(new float[]
                {fplane_right.getX(), fplane_bottom.getX(), fplane_center.getX(), camPos.getX(),
                        fplane_right.getY(), fplane_bottom.getY(), fplane_center.getY(), camPos.getY(),
                        fplane_right.getZ(), fplane_bottom.getZ(), fplane_center.getZ(), camPos.getZ(),
                        0, 0, 0, 1}
        );
    }

    /**
     * This converts the given Bounding Box coordinate passed with a 2d vector into a 3d coordinate on the XZ plane
     * @param bb_pos
     * @return XZ & bb_pos intersection
     */
    public DepreciatedVector3F convertIMGCoord(DepreciatedVector2F bb_pos){
        DepreciatedVector3F castedVector = this.imgToLocal.matMul(new DepreciatedVector4F(bb_pos.getX(), bb_pos.getY(), 1, 1)).getAsVec3f();
        DepreciatedVector3F output = this.cardinalAxisPlane.getVector3fInt(camPos, castedVector);
        return output;
    }

    /**
     * This converts the given Bounding Box coordinate passed with a 2d vector into a 3d coordinate on the XZ plane (4D input version)
     * @param bb_pos
     * @return XZ & bb_pos intersection
     */
    public DepreciatedVector3F convertIMGCoord(DepreciatedVector4F bb_pos){
        DepreciatedVector3F output = this.cardinalAxisPlane.getVector3fInt(camPos, this.imgToLocal.matMul(bb_pos).getAsVec3f());
        return output;
    }

    @Override
    public String toString(){
        return "HFOV: " + this.hFOV + " \nVFOV: " + this.vFOV + " \nCamera Position Vector3f: " + this.camPos.toString() + " \nCamera Rotation Matrix: \n" + this.camRot.toString();
    }

    /**
     * GETTERS AND SETTERS
     */

    public void setCamRot(DepreciatedMatrix4f newRotation){
        this.camRot = newRotation;
        this.setupFrustum();
    }

    public void setCamPos(DepreciatedVector3F newPosition){
        this.camPos = newPosition;
        this.setupFrustum();
    }

    public DepreciatedMatrix4f getImgToLocal() {
        return imgToLocal;
    }

    public DepreciatedMatrix4f getCamRot() {
        return camRot;
    }

    public double gethFOV() {
        return hFOV;
    }

    public double getvFOV() {
        return vFOV;
    }

    public DepreciatedVector4F getFplane_right() {
        return fplane_right;
    }

    public DepreciatedVector4F getFplane_bottom() {
        return fplane_bottom;
    }

    public DepreciatedVector4F getFplane_center() {
        return fplane_center;
    }

    public DepreciatedVector3F getCamPos() {
        return camPos;
    }

    public DepreciatedPlane3f getCardinalAxisPlane() {
        return cardinalAxisPlane;
    }
}
