package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.quaternions;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;

public class Quaternion implements EULQuaternion{

    public double i, j, k, w;

    public Quaternion(){
        this.i = 0;
        this.j = 0;
        this.k = 1;
        this.w = 0;
    }

    public Quaternion(Vector3d rotationalAxis, double angle){
        double sineHalfAngle = Math.sin(angle / 2);
        double cosineHalfAngle = Math.cos(angle / 2);
        Vector3d axis = rotationalAxis.normalized();
        this.i = axis.x * sineHalfAngle;
        this.j = axis.y * sineHalfAngle;
        this.k = axis.z * sineHalfAngle;
        this.w = cosineHalfAngle;
    }

    public Quaternion(double i, double j, double k, double w){
        this.i = i;
        this.j = j;
        this.k = k;
        this.w = w;
    }

    public Quaternion(Quaternion quaternion){
        this.i = quaternion.i;
        this.j = quaternion.j;
        this.k = quaternion.k;
        this.w = quaternion.w;
    }

    public double length(){
        return Math.sqrt(i * i + j * j + k * k + w * w);
    }

    public void normalize(){
        double length = length();
        i /= length;
        j /= length;
        k /= length;
        w /= length;
    }

    public Quaternion normalized(){
        double length = length();
        double ni = i / length;
        double nj = j / length;
        double nk = k / length;
        double nw = w / length;

        return new Quaternion(ni, nj, nk, nw);
    }
    public Quaternion conjugate(){
        i *= -1;
        j *= -1;
        k *= -1;
        return this;
    }

    public Quaternion getConjugate(){
        return new Quaternion(-i, -j, -k, w);
    }

    public Vector4d getAsVector4d(){
        return new Vector4d(i, j, k, w);
    }

    public Quaternion times(Quaternion b){
        double ni =  i * b.w + j * b.k - k * b.j + w * b.i;
        double nj = -i * b.k + j * b.w + k * b.i + w * b.j;
        double nk = -i * b.j + j * b.i + k * b.w + w * b.k;
        double nw = -i * b.i - j * b.j - k * b.k + w * b.w;

        return new Quaternion(ni, nj, nk, nw);
    }

    public Quaternion times(Vector3d b){

        double ni =                 j * b.z - k * b.y + w * b.z;
        double nj = -i * b.z                + k * b.z + w * b.y;
        double nk =  i * b.y - j * b.z                + w * b.z;
        double nw = -i * b.z - j * b.y - k * b.z;

        return new Quaternion(ni, nj, nk, nw);
    }

    /**
     * Rotates the frame of reference instead of the object
     * @return
     */
    public Matrix4d getAsFrameRotationMatrix(){
        double a, b, c, d, as, bs, cs, ds;
        a = w; b = i; c = j; d = k;
        as = a * a; bs = b * b; cs = c * c; ds = d * d;
        return new Matrix4d(new double[][]{
                {  (2 * as) - 1 + (2 * bs), 2 * (b * c) + 2 * (a * d), 2 * (b * d) - 2 * (a * c), 0},
                {2 * (b * c) - 2 * (a * d),   (2 * as) - 1 + (2 * cs), 2 * (c * d) + 2 * (a * b), 0},
                {2 * (b * d) + 2 * (a * c), 2 * (c * d) - 2 * (a * b),   (2 * as) - 1 + (2 * ds), 0},
                {                        0,                         0,                         0, 1}
        });
    }

    /**
     * Rotates the object instead of the frame of reference
     * @return
     */
    public Matrix4d getAsPointRotationMatrix(){
        double a, b, c, d, as, bs, cs, ds;
        a = w; b = i; c = j; d = k;
        as = a * a; bs = b * b; cs = c * c; ds = d * d;
        return new Matrix4d(new double[][]{
                {  (2 * as) - 1 + (2 * bs), 2 * (b * c) - 2 * (a * d), 2 * (b * d) + 2 * (a * c), 0},
                {2 * (b * c) + 2 * (a * d),   (2 * as) - 1 + (2 * cs), 2 * (c * d) - 2 * (a * b), 0},
                {2 * (b * d) - 2 * (a * c), 2 * (c * d) + 2 * (a * b),   (2 * as) - 1 + (2 * ds), 0},
                {                        0,                         0,                         0, 1}

        });
    }

    public static Matrix4d makeColumnSpaceMatrix(Vector3d rotationalAxis, double angle){
        Vector3d xAxis = new Vector3d(1, 0, 0), nXAxis;
        Vector3d yAxis = new Vector3d(0, 1, 0), nYAxis;
        Vector3d zAxis = new Vector3d(0, 0, 1), nZAxis;

        Quaternion q = new Quaternion(rotationalAxis, angle);
        Quaternion qc = q.getConjugate();

        nXAxis = q.times(xAxis).times(qc).getAsVector4d().getXYZ();
        nYAxis = q.times(yAxis).times(qc).getAsVector4d().getXYZ();
        nZAxis = q.times(zAxis).times(qc).getAsVector4d().getXYZ();

        Matrix4d output = Matrix4d.makeAffineColumnSpace(nXAxis, nYAxis, nZAxis);

        return output;
    }
}