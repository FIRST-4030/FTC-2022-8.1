package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.MathEx;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;

import java.io.IOException;

/*
[0,  1, 2, 3]
[4,  5, 6, 7]
[8,  9,10,11]
[12,13,14,15]
*/
public class DepreciatedMatrix4f {

    private float[] m;

    public DepreciatedMatrix4f(){
        m = new float[] {1.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 1.0f};
    }

    public DepreciatedMatrix4f(float[] newMatrix){
        if (newMatrix.length == 16){
            m = newMatrix;
        } else {
            IOException e = new IOException("Matrix Length is not 16!");
            e.printStackTrace();
        }
    }
    /*********************************************************************************/
    public void add(DepreciatedMatrix4f b){
        for (int i = 0; i < 16; i++){
            this.m[i] += b.m[i];
        }
    }

    public void mul(float b){
        for (int i = 0; i < 16; i++){
            this.m[i] *= b;
        }
    }

    public DepreciatedVector4F matMul(DepreciatedVector4F a){
        float nx = a.x * m[0] +  a.y * m[1] +  a.z * m[2] +  a.w * m[3];
        float ny = a.x * m[4] +  a.y * m[5] +  a.z * m[6] +  a.w * m[7];
        float nz = a.x * m[8] +  a.y * m[9] +  a.z * m[10] + a.w * m[11];
        float nw = a.x * m[12] + a.y * m[12] + a.z * m[13] + a.w * m[14];
        return new DepreciatedVector4F(nx, ny, nz, nw);
    }
    /*********************************************************************************/
    public static DepreciatedMatrix3f add(DepreciatedMatrix4f a, DepreciatedMatrix4f b){
        float[] newMat = new float[16];
        for (int i = 0; i < 9; i++){
            newMat[i] = a.m[i] + b.m[i];
        }

        return new DepreciatedMatrix3f(newMat);
    }

    public static DepreciatedMatrix3f mul(DepreciatedMatrix4f a, float b){
        float[] newMat = new float[16];
        for (int i = 0; i < 9; i++){
            newMat[i] = a.m[i] * b;
        }

        return new DepreciatedMatrix3f(newMat);
    }

    public static DepreciatedMatrix4f matMul(DepreciatedMatrix4f a, DepreciatedMatrix4f b){
        float[][] cols = new float[][] {b.getCol(0), b.getCol(1), b.getCol(2), b.getCol(3)};
        float[][] rows = new float[][] {a.getRow(0), a.getRow(1), a.getRow(2), a.getRow(3)};
        float[] output = new float[16];

        for (int i = 0; i < 3; i++) {
            output[0 + i * 4] = cols[0][0] * rows[i][0] + cols[0][1] * rows[i][1] + cols[0][2] * rows[i][2] + cols[0][3] * rows[i][3];
            output[1 + i * 4] = cols[1][0] * rows[i][0] + cols[1][1] * rows[i][1] + cols[1][2] * rows[i][2] + cols[1][3] * rows[i][3];
            output[2 + i * 4] = cols[2][0] * rows[i][0] + cols[2][1] * rows[i][1] + cols[2][2] * rows[i][2] + cols[2][3] * rows[i][3];
            output[3 + i * 4] = cols[3][0] * rows[i][0] + cols[3][1] * rows[i][1] + cols[3][2] * rows[i][2] + cols[3][3] * rows[i][3];
        }

        return new DepreciatedMatrix4f(output);
    }
    /*********************************************************************************/

    public float det(){
        float a = MathEx.quick3fArrDet(new float[] {m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]}) * m[0];
        float b = MathEx.quick3fArrDet(new float[] {m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]}) * m[1];
        float c = MathEx.quick3fArrDet(new float[] {m[4], m[5], m[7], m[8], m[9],  m[11], m[12], m[13], m[15]}) * m[2];
        float d = MathEx.quick3fArrDet(new float[] {m[4], m[5], m[6], m[8], m[9],  m[10], m[12], m[13], m[14]}) * m[3];

        return a - b + c - d;
    }

    public void transpose(){
        float[] out = {m[0], m[4],  m[8], m[12],
                       m[1], m[5],  m[9], m[13],
                       m[2], m[6], m[10], m[14],
                       m[3], m[7], m[11], m[15]};
        m = out;
    }

    public void invert(){
        if (this.det() == 0){return;}
        float det_inv = 1 / this.det();

        float[] cofactor_mat = {this.getMinorDet(0), -this.getMinorDet(4), this.getMinorDet(8), -this.getMinorDet(12),
                                -this.getMinorDet(1), this.getMinorDet(5), -this.getMinorDet(9), this.getMinorDet(13),
                                this.getMinorDet(2), -this.getMinorDet(6), this.getMinorDet(10), -this.getMinorDet(14),
                                -this.getMinorDet(3), this.getMinorDet(7), -this.getMinorDet(11), this.getMinorDet(15)};

        for (int i = 0; i < cofactor_mat.length; i++){
            cofactor_mat[i] *= det_inv;
        }

        this.m = cofactor_mat;
    }
    /*********************************************************************************/
    public float[] getRow(int idx){
        return new float[] {m[idx * 4], m[idx * 4 + 1], m[idx * 4 + 2], m[idx * 4 + 3]};
    }

    public float[] getCol(int idx){
        return new float[] {m[idx], m[idx + 4], m[idx + 8], m[idx + 12]};
    }

    public float getMinorDet(int element_idx){
        int col = element_idx % 4, row = element_idx / 4, l = 0;
        float[] out3f = new float[9];

        for (int ex = 0; ex < 4; ex++){
            for (int ey = 0; ey < 4; ey++){
                if (ex != col && ey != row){
                    out3f[l] = m[ex + ey * 4];
                    l++;
                }
            }
        }

        return MathEx.quick3fArrDet(out3f);
    }

    public float[] getAsFloatArray() {return this.m;}

    public DepreciatedMatrix4f getAsTranspose(){
        return new DepreciatedMatrix4f(new float[]
                {m[0], m[4], m[8],  m[12],
                 m[1], m[5], m[9],  m[13],
                 m[2], m[6], m[10], m[14],
                 m[3], m[7], m[11], m[15],});
    }

    public DepreciatedMatrix4f getAsInversion(){
        if (this.det() == 0){return null;}
        float det_inv = 1 / this.det();

        float[] cofactor_mat = {this.getMinorDet(0), -this.getMinorDet(4),  this.getMinorDet(8),  -this.getMinorDet(12),
                                -this.getMinorDet(1), this.getMinorDet(5), -this.getMinorDet(9),  this.getMinorDet(13),
                                this.getMinorDet(2), -this.getMinorDet(6),  this.getMinorDet(10), -this.getMinorDet(14),
                                -this.getMinorDet(3), this.getMinorDet(7), -this.getMinorDet(11), this.getMinorDet(15)};

        for (int i = 0; i < cofactor_mat.length; i++){
            cofactor_mat[i] *= det_inv;
        }

        return new DepreciatedMatrix4f(cofactor_mat);
    }
    /*********************************************************************************/
    @Override
    public String toString(){
       return ("\n" +  m[0] + " " +  m[1] + " " +  m[2] + " " +  m[3] +
               "\n" +  m[4] + " " +  m[5] + " " +  m[6] + " " +  m[7] +
               "\n" +  m[8] + " " +  m[9] + " " + m[10] + " " + m[11] +
               "\n" + m[12] + " " + m[13] + " " + m[14] + " " + m[15]);
    }
}