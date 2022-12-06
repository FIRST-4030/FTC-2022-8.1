package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.DepreciatedPlane3f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.MathEx;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;

public class CameraLens {
    //For the isBusy() method
    private boolean busy = false;

    //Camera Attributes
    private double hFOV = 1, vFOV = 1; //initialize with default values in just in case
    private final double zFar = 1000; //set the far plane/ will stretch the Z coordinate
    private final DepreciatedPlane3f xzPlane = DepreciatedPlane3f.XZ_PLANE; //define the plane for clipping the Camera Position to Calculated Vector

    //NDC Img Attributes
    private DepreciatedVector4F imgRight = new DepreciatedVector4F(1, 0, 0, 1);
    private DepreciatedVector4F imgBottom = new DepreciatedVector4F(0, 1, 0, 1);
    private DepreciatedVector4F imgCenter = new DepreciatedVector4F(0, 0, 1, 1);

    //Describing transforms, most importantly, the rotation and translation
    private DepreciatedMatrix4f rotation = new DepreciatedMatrix4f();
    private DepreciatedVector3F translation = new DepreciatedVector3F();

    //Final output Matrix4f
    private DepreciatedMatrix4f imgToLocal = new DepreciatedMatrix4f();

    //Preset FOV value for the listed cameras (only the C270)
    public static final double[] C270_FOV = MathEx.findFOV(3.58, 2.02, 4.11);

    public CameraLens(double[] fov){
        this.hFOV = fov[0];
        this.vFOV = fov[1];
    }

    public CameraLens(double[] fov, DepreciatedVector3F lensPos, DepreciatedMatrix4f lensRot){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.translation = lensPos;
        this.rotation = lensRot;
    }

    public void setupCameraMatrix(){
        busy = true;

        //find half of the fov or aov
        double hHFOV = this.hFOV * 0.5;
        double hVFOV = this.vFOV * 0.5;

        //find the x and y scale for the camera fov
        double xScale = Math.tan(hHFOV) * zFar;
        double yScale = Math.tan(hVFOV) * zFar;

        //set the appropriate vectors with proper (X, Y, Z) scale
        this.imgRight.setX((float) xScale);
        this.imgBottom.setY((float) -yScale);
        this.imgCenter.setZ((float) this.zFar);

        //rotate those previous vectors
        this.imgRight = this.rotation.matMul(this.imgRight);
        this.imgBottom = this.rotation.matMul(this.imgBottom);
        this.imgCenter = this.rotation.matMul(this.imgCenter);

        //using a property of Matrices (columns bending the x, y, z... space)
        this.imgToLocal = new DepreciatedMatrix4f(
                new float[]{this.imgRight.getX(), this.imgBottom.getX(), this.imgCenter.getX(), this.translation.getX(),
                            this.imgRight.getY(), this.imgBottom.getY(), this.imgCenter.getY(), this.translation.getY(),
                            this.imgRight.getZ(), this.imgBottom.getZ(), this.imgCenter.getZ(), this.translation.getZ(),
                                               0,                     0,                     0,                       1}
        );

        busy = false;
    }

    public DepreciatedVector3F findImgToLocal(DepreciatedVector2F imgCoordinate){
        busy = true;
        DepreciatedVector3F preClippedVector = imgToLocal.matMul(new DepreciatedVector4F(imgCoordinate.getX(), imgCoordinate.getY(), 1, 1)).getAsVec3f();
        DepreciatedVector3F output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public DepreciatedVector3F findImgToLocal(float imgX, float imgY){
        busy = true;
        DepreciatedVector3F preClippedVector = imgToLocal.matMul(new DepreciatedVector4F(imgX, imgY, 1, 1)).getAsVec3f();
        DepreciatedVector3F output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public void setTranslation(DepreciatedVector3F newPosition){
        this.translation = newPosition;
        this.setupCameraMatrix();
    }

    public void setRotation(DepreciatedMatrix4f newRotation){
        this.rotation = newRotation;
        this.setupCameraMatrix();
    }

    public double getZFar(){
        return this.zFar;
    }

    public double getHFOV(){
        return this.hFOV;
    }

    public double getVFOV(){
        return this.vFOV;
    }

    public DepreciatedVector3F getTranslation(){
        return this.translation;
    }

    public DepreciatedVector4F getImgRight(){
        return this.imgRight;
    }

    public DepreciatedVector4F getImgBottom(){
        return this.imgBottom;
    }

    public DepreciatedVector4F getImgCenter(){
        return this.imgCenter;
    }

    public DepreciatedPlane3f getXZPlane(){
        return this.xzPlane;
    }

    public DepreciatedMatrix4f getRotation(){
        return this.rotation;
    }

    public DepreciatedMatrix4f getImgToLocal(){
        return this.imgToLocal;
    }

    @Override
    public String toString(){
        return  "CameraLens Attributes: " +
                "\nHFov: " + this.hFOV +
                "\nVFov: " + this.vFOV +
                "\nImgRight: " + this.imgRight +
                "\nImgBottom: " + this.imgBottom +
                "\nImgCenter: " + this.imgCenter +
                "\nTranslation: " + this.translation.toString() +
                "\nRotation: \n" + this.rotation.toString() +
                "\nImgToLocalMatrix: \n" + this.imgToLocal.toString();
    }

    public boolean isBusy() {
        return busy;
    }

    public static class CameraLensAlt{

        public enum LERP_SCHEMA{
            TC_TL_C_L,
            C_L_BC_BL,
            TC_TR_C_R,
            C_R_BC_BR
        }

        private DepreciatedVector3F TL, TC, TR,
                          L,  C,  R,
                         BL, BC, BR;

        private DepreciatedVector3F topX1, topX2, botX1, botX2;

        private LERP_SCHEMA schema;

        public CameraLensAlt(){
            initVector2fs();
        }

        public DepreciatedVector3F calcImgToLocal(float imgX, float imgY){
            DepreciatedVector3F output = new DepreciatedVector3F();
            DepreciatedVector3F topY = new DepreciatedVector3F();
            DepreciatedVector3F botY = new DepreciatedVector3F();

            switch (this.schema){
                case TC_TL_C_L:
                case C_L_BC_BL:
                    topY = MathEx.lerp(topX1, topX2, -imgX);
                    botY = MathEx.lerp(botX1, botX2, -imgX);
                    output = MathEx.lerp(botY, topY, -imgY);
                    break;
                case TC_TR_C_R:
                case C_R_BC_BR:
                    topY = MathEx.lerp(topX1, topX2, imgX);
                    botY = MathEx.lerp(botX1, botX2, imgX);
                    output = MathEx.lerp(topY, botY, imgY);
                    break;
            }

            return output;
        }

        private void initVector2fs(){
            //init top Vec2fs
            TL = new DepreciatedVector3F();
            TC = new DepreciatedVector3F();
            TR = new DepreciatedVector3F();

            //init mid Vec2fs
            L = new DepreciatedVector3F();
            C = new DepreciatedVector3F();
            R = new DepreciatedVector3F();

            //init bot Vec2fs
            BL = new DepreciatedVector3F();
            BC = new DepreciatedVector3F();
            BR = new DepreciatedVector3F();

            //init lerp vectors
            topX1 = new DepreciatedVector3F();
            topX2 = new DepreciatedVector3F();
            botX1 = new DepreciatedVector3F();
            botX2 = new DepreciatedVector3F();
        }

        public void setSchema(LERP_SCHEMA newSchema){
            this.schema = newSchema;
            updateVectors();
        }

        public void updateVectors(){
            switch (this.schema){
                case C_R_BC_BR:
                    topX1 = C;
                    topX2 = R;
                    botX1 = BC;
                    botX2 = BR;
                    break;

                case TC_TR_C_R:
                    topX1 = TC;
                    topX2 = TR;
                    botX1 = C;
                    botX2 = R;
                    break;

                case C_L_BC_BL:
                    topX1 = C;
                    topX2 = L;
                    botX1 = BC;
                    botX2 = BL;
                    break;
                case TC_TL_C_L:
                    topX1 = TC;
                    topX2 = TL;
                    botX1 = C;
                    botX2 = L;
                    break;
            }
        }

        public DepreciatedVector3F getTL() {
            return TL;
        }

        public void setTL(DepreciatedVector3F TL) {
            this.TL = TL;
            updateVectors();
        }

        public DepreciatedVector3F getTC() {
            return TC;
        }

        public void setTC(DepreciatedVector3F TC) {
            this.TC = TC;
            updateVectors();
        }

        public DepreciatedVector3F getTR() {
            return TR;
        }

        public void setTR(DepreciatedVector3F TR) {
            this.TR = TR;
            updateVectors();
        }

        public DepreciatedVector3F getL() {
            return L;
        }

        public void setL(DepreciatedVector3F l) {
            L = l;
            updateVectors();
        }

        public DepreciatedVector3F getC() {
            return C;
        }

        public void setC(DepreciatedVector3F c) {
            C = c;
            updateVectors();
        }

        public DepreciatedVector3F getR() {
            return R;
        }

        public void setR(DepreciatedVector3F r) {
            R = r;
            updateVectors();
        }

        public DepreciatedVector3F getBL() {
            return BL;
        }

        public void setBL(DepreciatedVector3F BL) {
            this.BL = BL;
            updateVectors();
        }

        public DepreciatedVector3F getBC() {
            return BC;
        }

        public void setBC(DepreciatedVector3F BC) {
            this.BC = BC;
            updateVectors();
        }

        public DepreciatedVector3F getBR() {
            return BR;
        }

        public void setBR(DepreciatedVector3F BR) {
            this.BR = BR;
            updateVectors();
        }
    }
}
