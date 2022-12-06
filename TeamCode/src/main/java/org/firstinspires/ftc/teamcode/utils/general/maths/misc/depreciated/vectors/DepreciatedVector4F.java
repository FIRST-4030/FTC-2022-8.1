package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors;

import java.io.IOException;

public class DepreciatedVector4F implements DepreciatedVectorImpl {
    public static final DepreciatedVector4F instance = new DepreciatedVector4F(0, 0, 0, 0);
    public float x, y, z, w;

    public DepreciatedVector4F(){
        x = 0; y = 0; z = 0; w = 1;
    }

    public DepreciatedVector4F(float[] list){
        if (list.length == 4){
            x = list[0]; y = list[1]; z = list[2]; w = list[3];
        } else {
            IOException e = new IOException("Array length is not 4!");
            e.printStackTrace();
        }
    }

    public DepreciatedVector4F(float x, float y, float z){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = 1;
    }

    public DepreciatedVector4F(float x, float y, float z, float w){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public float getX(){
        return this.x;
    }

    public float getY(){
        return this.y;
    }

    public float getZ(){
        return this.z;
    }

    public float getW(){
        return this.w;
    }

    public void setX(float nx){
        this.x = nx;
    }

    public void setY(float ny){
        this.y = ny;
    }

    public void setZ(float nz){
        this.z = nz;
    }

    public void setW(float nw){
        this.w = nw;
    }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
    }

    public void normalize(){
        float length = this.length();
        this.x /= length;
        this.y /= length;
        this.z /= length;
        this.w /= length;
    }

    public DepreciatedVector4F normalized(){
        float length = this.length();
        float nx = this.x, ny = this.y,  nz = this.z,  nw = this.w;
        nx /= length;
        ny /= length;
        nz /= length;
        nw /= length;
        return new DepreciatedVector4F(nx, ny, nz, nw);
    }


    public void add(DepreciatedVector4F b){
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
        this.w += b.w;
    }

    public void sub(DepreciatedVector4F b){
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
        this.w -= b.w;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
        this.z *= b;
        this.w *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
        this.z /= b;
        this.w /= b;
    }

    public float dot(DepreciatedVector4F b){
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public static DepreciatedVector4F add(DepreciatedVector4F a, DepreciatedVector4F b){
        return new DepreciatedVector4F(a.x + b.x ,a.y + b.y, a.z + b.z, a.w + b.w);
    }

    public static DepreciatedVector4F sub(DepreciatedVector4F a, DepreciatedVector4F b){
        return new DepreciatedVector4F(a.x - b.x ,a.y - b.y, a.z - b.z, a.w - b.w);
    }

    public static DepreciatedVector4F mul(DepreciatedVector4F a, float b){
        return new DepreciatedVector4F(a.x * b, a.y * b, a.z * b, a.w * b);
    }

    public static DepreciatedVector4F div(DepreciatedVector4F a, float b){
        return new DepreciatedVector4F(a.x / b, a.y / b, a.z / b, a.w / b);
    }

    public static float dot(DepreciatedVector4F a, DepreciatedVector4F b){
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    public DepreciatedVector3F getAsVec3f(){
        return new DepreciatedVector3F(this.x, this.y, this.z);
    }

    @Override
    public String toString(){ return ("X: " + this.x + " Y: " + this.y + " Z: " + this.z + " W: " + this.w);}

    @Override
    public float[] getAsList() {
        return new float[]{x, y, z, w};
    }
}
