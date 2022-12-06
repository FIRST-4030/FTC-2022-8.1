package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors;

public class Vector2d implements EULVector<Vector2d> {

    public static Vector2d
            AXIS_X = new Vector2d(1, 0),
            AXIS_Y = new Vector2d(0, 1);

    public double x, y;

    public Vector2d(){
        this.x = 0.0;
        this.y = 0.0;
    }

    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    public String toString(){
        return "(" + x + ", " + y + ")";
    }

    @Override
    public double[] asArray() {
        return new double[]{x, y};
    }

    @Override
    public double get(int n) {
        double output;
        switch (n){
            case 0: output = this.x; break;
            case 1: output = this.y; break;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
        return output;
    }

    @Override
    public void set(int n, double v) {
        switch(n){
            case 0: x = v; break;
            case 1: y = v; break;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
    }

    @Override
    public double length() {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public void normalize() {
        double len = length();
        x /= len;
        y /= len;
    }

    @Override
    public Vector2d normalized() {
        double len = length();
        return new Vector2d(x / len, y / len);
    }

    @Override
    public Vector2d plus(Object other) {
        Vector2d other_ = other instanceof Vector2d ? (Vector2d) other : new Vector2d();
        return new Vector2d(x + other_.x, y + other_.y);
    }

    @Override
    public Vector2d minus(Object other) {
        Vector2d other_ = other instanceof Vector2d ? (Vector2d) other : new Vector2d();
        return new Vector2d(x - other_.x, y - other_.y);
    }

    @Override
    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }

    @Override
    public double times(Object other) {
        Vector2d other_ = other instanceof Vector2d ? (Vector2d) other : new Vector2d();
        return x * other_.x + y * other_.y;
    }

    @Override
    public Vector2d times(int other) {
        return new Vector2d(x * other, y * other);
    }

    @Override
    public Vector2d times(float other) {
        return new Vector2d(x * other, y * other);
    }

    @Override
    public Vector2d times(double other) {
        return new Vector2d(x * other, y * other);
    }

    @Override
    public Vector2d div(int other) {
        return new Vector2d(x / other, y / other);
    }

    @Override
    public Vector2d div(float other) {
        return new Vector2d(x / other, y / other);
    }

    @Override
    public Vector2d div(double other) {
        return new Vector2d(x / other, y / other);
    }

}
