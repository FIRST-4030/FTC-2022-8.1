package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors;

public class Vector3d implements EULVector<Vector3d> {

    public static Vector3d
            AXIS_X = new Vector3d(1, 0, 0),
            AXIS_Y = new Vector3d(0, 1, 0),
            AXIS_Z = new Vector3d(0, 0, 1);

    public double x, y, z;

    public Vector3d(){
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    public Vector3d(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    @Override
    public double[] asArray() {
        return new double[]{x, y, z};
    }

    public Vector2d getXY(){
        return new Vector2d(x, y);
    }

    @Override
    public double get(int n) {
        switch(n){
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
    }

    @Override
    public void set(int n, double v) {
        switch(n){
            case 0: x = v; break;
            case 1: y = v; break;
            case 2: z = v; break;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
    }

    @Override
    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    @Override
    public void normalize() {
        double len = length();
        x /= len;
        y /= len;
        z /= len;
    }

    @Override
    public Vector3d normalized() {
        double len = length();
        return new Vector3d(x / len, y / len, z / len);
    }

    public Vector3d cross(Vector3d other){
        return new Vector3d(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    @Override
    public Vector3d plus(Object other) {
        Vector3d other_ = other instanceof Vector3d ? (Vector3d) other : new Vector3d();
        return new Vector3d(x + other_.x, y + other_.y, z + other_.z);
    }

    @Override
    public Vector3d minus(Object other) {
        Vector3d other_ = other instanceof Vector3d ? (Vector3d) other : new Vector3d();
        return new Vector3d(x - other_.x, y - other_.y, z - other_.z);
    }

    @Override
    public Vector3d unaryMinus() {
        return new Vector3d(-x, -y, -z);
    }

    @Override
    public double times(Object other) {
        Vector3d other_ = other instanceof Vector3d ? (Vector3d) other : new Vector3d();
        return x * other_.x + y * other_.y + z * other_.z;
    }

    @Override
    public Vector3d times(int other) {
        return new Vector3d(x * other, y * other, z * other);
    }

    @Override
    public Vector3d times(float other) {
        return new Vector3d(x * other, y * other, z * other);
    }

    @Override
    public Vector3d times(double other) {
        return new Vector3d(x * other, y * other, z * other);
    }

    @Override
    public Vector3d div(int other) {
        return new Vector3d(x * other, y * other, z * other);
    }

    @Override
    public Vector3d div(float other) {
        return new Vector3d(x * other, y * other, z * other);
    }

    @Override
    public Vector3d div(double other) {
        return new Vector3d(x * other, y * other, z * other);
    }
}
