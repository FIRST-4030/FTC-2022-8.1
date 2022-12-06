package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors;

public class Vector4d implements EULVector<Vector4d> {

    public static Vector4d
            AXIS_X = new Vector4d(1, 0, 0, 0),
            AXIS_Y = new Vector4d(0, 1, 0, 0),
            AXIS_Z = new Vector4d(0, 0, 1, 0),
            AXIS_W = new Vector4d(0, 0, 0, 1);

    public double x, y, z, w;

    public Vector4d(){
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
        this.w = 0.0;
    }

    public Vector4d(double x, double y, double z, double w){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    @Override
    public String toString(){
        return "(" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    @Override
    public double[] asArray() {
        return new double[]{x, y, z, w};
    }

    public Vector3d getXYZ(){
        return new Vector3d(x, y, z);
    }

    @Override
    public double get(int n) {
        switch (n){
            case 0: return this.x;
            case 1: return this.y;
            case 2: return this.z;
            case 3: return this.w;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
    }

    @Override
    public void set(int n, double v) {
        switch (n){
            case 0: this.x = v; break;
            case 1: this.y = v; break;
            case 2: this.z = v; break;
            case 3: this.w = v; break;
            default: throw new ArrayIndexOutOfBoundsException("Index is out of bounds: " + n);
        }
    }

    @Override
    public double length() {
        return Math.sqrt(x * x + y * y + z * z + w * w);
    }

    @Override
    public void normalize() {
        double len = length();
        x /= len;
        y /= len;
        z /= len;
        w /= len;
    }

    @Override
    public Vector4d normalized() {
        double len = length();
        return new Vector4d(x / len, y / len, z / len, w / len);
    }

    @Override
    public Vector4d plus(Object other) {
        Vector4d other_ = other instanceof Vector4d ? (Vector4d) other : new Vector4d();
        return new Vector4d(x + other_.x, y + other_.y, z + other_.z, w + other_.w);
    }

    @Override
    public Vector4d minus(Object other) {
        Vector4d other_ = other instanceof Vector4d ? (Vector4d) other : new Vector4d();
        return new Vector4d(x - other_.x, y - other_.y, z - other_.z, w - other_.w);
    }

    @Override
    public Vector4d unaryMinus() {
        return new Vector4d(-x, -y, -z, -w);
    }

    @Override
    public double times(Object other) {
        Vector4d other_ = other instanceof Vector4d ? (Vector4d) other : new Vector4d();
        return x * other_.x + y * other_.y + z * other_.z + w * other_.w;
    }

    @Override
    public Vector4d times(int other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }

    @Override
    public Vector4d times(float other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }

    @Override
    public Vector4d times(double other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }

    @Override
    public Vector4d div(int other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }

    @Override
    public Vector4d div(float other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }

    @Override
    public Vector4d div(double other) {
        return new Vector4d(x * other, y * other, z * other, w * other);
    }
}
