package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.io.IOException;

public class DepreciatedVector2F implements DepreciatedVectorImpl {
    public static final DepreciatedVector2F instance = new DepreciatedVector2F(0, 0);
    public float x, y;

    public DepreciatedVector2F(){
        this.x = 0;
        this.y = 0;
    }

    public DepreciatedVector2F(float x, float y){
        this.x = x;
        this.y = y;
    }

    public DepreciatedVector2F(float[] list){
        if (list.length == 2) {
            this.x = list[0];
            this.y = list[1];
        } else {
            IOException e = new IOException("Array length is not 2!");
            e.printStackTrace();
        }
    }

    public float getX(){
        return this.x;
    }

    public float getY(){
        return this.y;
    }

    public void setX(float nx){ this.x = nx; }

    public void setY(float ny){ this.y = ny; }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public void normalize(){
        float l = this.length();
        this.x /= l;
        this.y /= l;
    }

    public DepreciatedVector2F normalized(){
        float l = this.length();
        float nx = this.x / l;
        float ny = this.y / l;
        return new DepreciatedVector2F(nx, ny);
    }

    public void add(DepreciatedVector2F b){
        this.x += b.x;
        this.y += b.y;
    }

    public void sub(DepreciatedVector2F b){
        this.x -= b.x;
        this.y -= b.y;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
    }

    public float dot(DepreciatedVector2F b){
        return this.x * b.x + this.y * b.y;
    }

    public static DepreciatedVector2F add(DepreciatedVector2F a, DepreciatedVector2F b){
        return new DepreciatedVector2F(a.x + b.x ,a.y + b.y);
    }

    public static DepreciatedVector2F sub(DepreciatedVector2F a, DepreciatedVector2F b){
        return new DepreciatedVector2F(a.x - b.x ,a.y - b.y);
    }

    public static DepreciatedVector2F mul(DepreciatedVector2F a, float b){
        return new DepreciatedVector2F(a.x * b, a.y * b);
    }

    public static DepreciatedVector2F div(DepreciatedVector2F a, float b){
        return new DepreciatedVector2F(a.x / b, a.y / b);
    }

    public static float dot(DepreciatedVector2F a, DepreciatedVector2F b){
        return a.x * b.x + a.y * b.y;
    }

    public Vector2d getAsVector2d(){
        return new Vector2d(this.x, this.y);
    }

    @Override
    public String toString(){
        return ("X: " + this.x + " Y: " + this.y);
    }

    @Override
    public float[] getAsList() {
        return new float[]{x, y};
    }
}
