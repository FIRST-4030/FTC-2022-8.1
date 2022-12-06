package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors;

import java.io.IOException;

public class DepreciatedVector3F implements DepreciatedVectorImpl {
    public float x, y, z;

    public DepreciatedVector3F(){
        this.x = 0;
        this.y = 0;
        this.z = 1;
    }

    public DepreciatedVector3F(float x, float y, float z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public DepreciatedVector3F(float[] list){
        if (list.length == 3) {
            this.x = list[0];
            this.y = list[1];
            this.z = list[2];
        } else {
            IOException e = new IOException("Array length is not 3!");
            e.printStackTrace();
        }
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

    public void setX(float nx){ this.x = nx; }

    public void setY(float ny){ this.y = ny; }

    public void setZ(float nz){ this.z = nz; }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    public void normalize(){
        float l = this.length();
        this.x /= l;
        this.y /= l;
        this.z /= l;
    }

    public DepreciatedVector3F normalized(){
        float l = this.length();
        float nx = this.x / l;
        float ny = this.y / l;
        float nz = this.z / l;
        return new DepreciatedVector3F(nx, ny, nz);
    }

    public void add(DepreciatedVector3F b){
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
    }

    public void sub(DepreciatedVector3F b){
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
        this.z *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
        this.z /= b;
    }

    public float dot(DepreciatedVector3F b){
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public static DepreciatedVector3F add(DepreciatedVector3F a, DepreciatedVector3F b){
        return new DepreciatedVector3F(a.x + b.x ,a.y + b.y, a.z + b.z);
    }

    public static DepreciatedVector3F sub(DepreciatedVector3F a, DepreciatedVector3F b){
        return new DepreciatedVector3F(a.x - b.x ,a.y - b.y, a.z - b.z);
    }

    public static DepreciatedVector3F mul(DepreciatedVector3F a, float b){
        return new DepreciatedVector3F(a.x * b, a.y * b, a.z * b);
    }

    public static DepreciatedVector3F div(DepreciatedVector3F a, float b){
        return new DepreciatedVector3F(a.x / b, a.y / b, a.z / b);
    }

    public static float dot(DepreciatedVector3F a, DepreciatedVector3F b){
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    @Override
    public String toString(){
        return ("X: " + this.x + " Y: " + this.y + " Z: " + this.z);
    }

    @Override
    public float[] getAsList() {
        return new float[]{x, y, z};
    }
}