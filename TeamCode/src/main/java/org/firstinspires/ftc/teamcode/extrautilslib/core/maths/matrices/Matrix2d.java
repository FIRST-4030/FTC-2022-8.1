package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class Matrix2d implements EULMatrix<Matrix2d, Vector2d>{

    public double[][] matrix;

    public Matrix2d(){
        matrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
    }

    public Matrix2d(double[] firstRow, double[] secondRow){
        if (firstRow.length == 2 && secondRow.length == 2) {
            matrix = new double[][]{firstRow, secondRow};
        } else {
            matrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
        }
    }

    public Matrix2d(double[][] arr){
        if (arr.length == 2 && arr[0].length == 2) {
            matrix = arr;
        } else {
            matrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
        }
    }

    @Override
    public String toString() {
        String firstRow = "{" + matrix[0][0] + ", " + matrix[0][1] + "}";
        String secondRow = "{" + matrix[1][0] + ", " + matrix[1][1] + "}";
        return "{" + firstRow + secondRow + "}";
    }

    @Override
    public double get(int iy, int ix) {
        return matrix[iy][ix];
    }

    @Override
    public void set(int iy, int ix, double value) {
        matrix[iy][ix] = value;
    }

    @Override
    public Matrix2d plus(Matrix2d other) {
        return new Matrix2d(new double[][]{
                {matrix[0][0] + other.matrix[0][0], matrix[0][1] + other.matrix[0][1]},
                {matrix[1][0] + other.matrix[1][0], matrix[1][1] + other.matrix[1][1]}
        });
    }

    @Override
    public Matrix2d minus(Matrix2d other) {
        Matrix2d negativeOther = other.unaryMinus();
        return plus(negativeOther);
    }

    @Override
    public Matrix2d unaryMinus() {
        return new Matrix2d(new double[][]{
                {-matrix[0][0], -matrix[0][1]},
                {-matrix[1][0], -matrix[1][1]}
        });
    }

    @Override
    public Matrix2d times(double other) {
        return new Matrix2d(new double[][]{
                {matrix[0][0] * other, matrix[0][1] * other},
                {matrix[1][0] * other, matrix[1][1] * other}
        });
    }

    @Override
    public Matrix2d times(Matrix2d other) {
        return new Matrix2d(new double[][]{
                {matrix[0][0] * other.matrix[0][0] + matrix[0][1] * other.matrix[1][0], matrix[0][0] * other.matrix[0][1] + matrix[0][1] * other.matrix[1][1]},
                {matrix[1][0] * other.matrix[0][0] + matrix[1][1] * other.matrix[1][0], matrix[1][0] * other.matrix[0][1] + matrix[1][1] * other.matrix[1][1]}
        });
    }

    @Override
    public Vector2d times(Vector2d other) {
        return new Vector2d(matrix[0][0] * other.x + matrix[0][1] * other.y,
                            matrix[1][0] * other.x + matrix[1][1] * other.y);
    }

    @Override
    public double det() {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    }

    @Override
    public void transpose() {
        double[][] prev = matrix;
        matrix[0][1] = prev[1][0];
        matrix[1][0] = prev[0][1];
    }

    @Override
    public Matrix2d transposed() {
        return new Matrix2d(new double[][]{
                {matrix[0][0], matrix[1][0]},
                {matrix[0][1], matrix[1][1]}
        });
    }

    @Override
    public void invert() {
        double det = det();
        double[][] prev = matrix;
        if (det != 0){
            matrix = new double[][]{
                    {prev[1][1] / det, -prev[0][1] / det},
                    {-prev[1][0] / det, prev[0][0] / det}
            };
        }
    }

    @Override
    public Matrix2d inverted() {
        double det = det();
        double[][] m = matrix;
        if (det != 0){
            m = new double[][]{
                    {matrix[1][1] / det, -matrix[0][1] / det},
                    {-matrix[1][0] / det, matrix[0][0] / det}
            };
        }
        return new Matrix2d(m);
    }

    public static Matrix2d makeRotation(double angle){
        double rs = Math.sin(angle); double rc = Math.cos(angle);
        return new Matrix2d(new double[][]{
                {rc, -rs},
                {rs,  rc}
        });
    }
}
