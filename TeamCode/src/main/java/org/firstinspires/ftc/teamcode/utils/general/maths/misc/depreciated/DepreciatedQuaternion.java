package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;

public class DepreciatedQuaternion {

    private float i, j, k, w;

    public DepreciatedQuaternion(float i, float j, float k, float w){

    }

    public float length(){
        return (float) Math.sqrt(i * i + j * j + k * k + w * w);
    }

    public void normalize(){
        float length = this.length();
        i /= length;
        j /= length;
        k /= length;
        w /= length;
    }

    public DepreciatedQuaternion normalized(){
        float length = this.length();
        float ni = this.i / length;
        float nj = this.j / length;
        float nk = this.k / length;
        float nw = this.w / length;

        return new DepreciatedQuaternion(ni, nj, nk, nw);
    }

    public DepreciatedQuaternion conjugate(){
        return new DepreciatedQuaternion(-i, -j, -k, w);
    }

    public DepreciatedQuaternion mul(DepreciatedQuaternion b){
        float ni =  i * b.getW() + j * b.getK() - k * b.getJ() + w * b.getI();
        float nj = -i * b.getK() + j * b.getW() + k * b.getI() + w * b.getJ();
        float nk = -i * b.getJ() + j * b.getI() + k * b.getW() + w * b.getK();
        float nw = -i * b.getI() - j * b.getJ() - k * b.getK() + w * b.getW();

        return new DepreciatedQuaternion(ni, nj, nk, nw);
    }

    public DepreciatedQuaternion mul(DepreciatedVector3F b){

        float ni =                 j * b.getZ() - k * b.getY() + w * b.getX();
        float nj = -i * b.getZ()                + k * b.getX() + w * b.getY();
        float nk =  i * b.getY() - j * b.getX()                + w * b.getZ();
        float nw = -i * b.getX() - j * b.getY() - k * b.getZ();

        return new DepreciatedQuaternion(ni, nj, nk, nw);
    }

    public float getI() {
        return i;
    }

    public void setI(float i) {
        this.i = i;
    }

    public float getJ() {
        return j;
    }

    public void setJ(float j) {
        this.j = j;
    }

    public float getK() {
        return k;
    }

    public void setK(float k) {
        this.k = k;
    }

    public float getW() {
        return w;
    }

    public void setW(float w) {
        this.w = w;
    }

    /**
     * Rotates the frame of reference instead of the object
     * @return
     */
    public DepreciatedMatrix4f getAsFrameRotationMatrix(){
        float a, b, c, d, as, bs, cs, ds;
        a = w; b = i; c = j; d = k;
        as = a * a; bs = b * b; cs = c * c; ds = d * d;
        return new DepreciatedMatrix4f(
                new float[] {   (2 * as) - 1 + (2 * bs), 2 * (b * c) + 2 * (a * d), 2 * (b * d) - 2 * (a * c), 0,
                              2 * (b * c) - 2 * (a * d),   (2 * as) - 1 + (2 * cs), 2 * (c * d) + 2 * (a * b), 0,
                              2 * (b * d) + 2 * (a * c), 2 * (c * d) - 2 * (a * b),   (2 * as) - 1 + (2 * ds), 0,
                                                      0,                         0,                         0, 1
                });
    }

    /**
     * Rotates the object instead of the frame of reference
     * @return
     */
    public DepreciatedMatrix4f getAsPointRotationMatrix(){
        float a, b, c, d, as, bs, cs, ds;
        a = w; b = i; c = j; d = k;
        as = a * a; bs = b * b; cs = c * c; ds = d * d;
        return new DepreciatedMatrix4f(
                new float[] {  (2 * as) - 1 + (2 * bs), 2 * (b * c) - 2 * (a * d), 2 * (b * d) + 2 * (a * c), 0,
                             2 * (b * c) + 2 * (a * d),   (2 * as) - 1 + (2 * cs), 2 * (c * d) - 2 * (a * b), 0,
                             2 * (b * d) - 2 * (a * c), 2 * (c * d) + 2 * (a * b),   (2 * as) - 1 + (2 * ds), 0,
                                                     0,                         0,                         0, 1

                });
    }
}
