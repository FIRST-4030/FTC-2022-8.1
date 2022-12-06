package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;

import java.io.IOException;
/*
[0, 1]
[2, 3]
*/
public class DepreciatedMatrix2f {

    private float[] m;

    public DepreciatedMatrix2f() {
        m = new float[]{1.0f, 0.0f,
                        0.0f, 1.0f};
    }

    public DepreciatedMatrix2f(float[] matrix) {
        if (matrix.length == 4) {
            m = matrix;
        } else {
            IOException e = new IOException("Array length is not 4!");
            e.printStackTrace();
        }
    }
    /*********************************************************************************/
    public static DepreciatedMatrix2f add(DepreciatedMatrix2f a, DepreciatedMatrix2f b) {
        float[] am = a.getAsFloatArray();
        float[] bm = b.getAsFloatArray();
        return new DepreciatedMatrix2f(new float[]{am[0] + bm[0], am[1] + bm[1], am[2] + bm[2], am[3] + bm[3]});
    }

    public static DepreciatedMatrix2f mul(DepreciatedMatrix2f a, float factor) {
        float[] am = a.getAsFloatArray();
        return new DepreciatedMatrix2f(new float[]{am[0] * factor, am[1] * factor, am[2] * factor, am[3] * factor});
    }

    public static DepreciatedMatrix2f matMul(DepreciatedMatrix2f a, DepreciatedMatrix2f b) {
        float[] am = a.getAsFloatArray();
        float[] bm = b.getAsFloatArray();
        float e1, e2, e3, e4;
        e1 = am[0] * bm[0] + am[1] * bm[2];
        e2 = am[0] * bm[1] + am[1] * bm[3];
        e3 = am[2] * bm[0] + am[3] * bm[2];
        e4 = am[2] * bm[1] + am[3] * bm[3];
        return new DepreciatedMatrix2f(new float[]{e1, e2, e3, e4});
    }
    /*********************************************************************************/
    public void add(DepreciatedMatrix2f b) {
        for (int i = 0; i < m.length; i++) {
            m[i] += b.m[i];
        }
    }

    public void mul(float factor) {
        for (int i = 0; i < m.length; i++) {
            m[i] *= factor;
        }
    }

    public DepreciatedVector2F matMul(DepreciatedVector2F a) {
        float nX = a.x * m[0] + a.y * m[1];
        float nY = a.x * m[2] + a.y * m[3];
        return new DepreciatedVector2F(nX, nY);
    }
    /*********************************************************************************/
    public float det() { return m[0] * m[3] - m[1] * m[2]; }

    public void transpose() {
        float[] nm = m;
        m = new float[]{nm[0], nm[2], nm[1], nm[3]};
    }

    public void invert() {
        float det = this.det();
        if (det == 0) {
            return;
        }
        float[] nm = {m[3] / det, -m[1] / det, -m[2] / det, m[0] / det};
        m = nm;
    }
    /*********************************************************************************/
    public float[] getRow(int idx){
        return new float[]{m[idx * 2], m[idx * 2 + 1]};
    }

    public float[] getCol(int idx){
        return new float[]{m[idx], m[idx + 2]};
    }

    public float[] getAsFloatArray() { return m; }

    public DepreciatedMatrix2f getAsTranspose() {
        return new DepreciatedMatrix2f(new float[]
                {m[0], m[2],
                 m[1], m[3]});
    }

    public DepreciatedMatrix2f getAsInverse() {
        float det = this.det();
        if (det == 0) {
            return null;
        }
        return new DepreciatedMatrix2f(new float[]{m[3] / det, -m[1] / det, -m[2] / det, m[1] / det});
    }
    /*********************************************************************************/
    @Override
    public String toString(){
        return "\n" + m[0] + " " + m[1] + "\n" + m[2] + " " + m[3];
    }
}
