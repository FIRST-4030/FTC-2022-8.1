package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;

public class Matrix3d implements EULMatrix<Matrix3d, Vector3d>{

    public double[][] matrix;

    public Matrix3d(){
        matrix = new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    }

    public Matrix3d(double[] firstRow, double[] secondRow, double[] thirdRow){
        if (firstRow.length == 3 && secondRow.length == 3 && thirdRow.length == 3) {
            matrix = new double[][]{firstRow, secondRow, thirdRow};
        } else {
            matrix = new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        }
    }

    public Matrix3d(double[][] arr){
        if (arr.length == 3 && arr[0].length == 3) {
            matrix = arr;
        } else {
            matrix = new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        }
    }

    @Override
    public String toString() {
        String firstRow = "{" + matrix[0][0] + ", " + matrix[0][1] + ", " + matrix[0][2] + "}, ";
        String secondRow = "{" + matrix[1][0] + ", " + matrix[1][1] + ", " + matrix[1][2] + "}, ";
        String thirdRow = "{" + matrix[2][0] + ", " + matrix[2][1] + ", " + matrix[2][2] + "}";
        return "{" + firstRow + secondRow + thirdRow + "}";
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
    public Matrix3d plus(Matrix3d other) {
        return new Matrix3d(new double[][]{
                {matrix[0][0] + other.matrix[0][0], matrix[0][1] + other.matrix[0][1], matrix[0][2] + other.matrix[0][2]},
                {matrix[1][0] + other.matrix[1][0], matrix[1][1] + other.matrix[1][1], matrix[1][2] + other.matrix[1][2]},
                {matrix[2][0] + other.matrix[2][0], matrix[2][1] + other.matrix[2][1], matrix[2][2] + other.matrix[2][2]}
        });
    }

    @Override
    public Matrix3d minus(Matrix3d other) {
        Matrix3d negativeOther = other.unaryMinus();
        return plus(negativeOther);
    }

    @Override
    public Matrix3d unaryMinus() {
        return new Matrix3d(new double[][]{
                {-matrix[0][0], -matrix[0][1], -matrix[0][2]},
                {-matrix[1][0], -matrix[1][1], -matrix[1][2]},
                {-matrix[2][0], -matrix[2][1], -matrix[2][2]}
        });
    }

    @Override
    public Matrix3d times(double other) {
        return new Matrix3d(new double[][]{
                {matrix[0][0] * other, matrix[0][1] * other, matrix[0][2] * other},
                {matrix[1][0] * other, matrix[1][1] * other, matrix[1][2] * other},
                {matrix[2][0] * other, matrix[2][1] * other, matrix[2][2] * other}
        });
    }

    @Override
    public Matrix3d times(Matrix3d other) {
        double[][] nMatrix = new double[3][3];

        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix[m][n] = matrix[m][0] * other.matrix[0][n] + matrix[m][1] * other.matrix[1][n] + matrix[m][2] * other.matrix[2][n];
            }
        }

        return new Matrix3d(nMatrix);
    }

    @Override
    public Vector3d times(Vector3d other) {
        return new Vector3d(matrix[0][0] * other.x + matrix[0][1] * other.y + matrix[0][2] * other.z,
                            matrix[1][0] * other.x + matrix[1][1] * other.y + matrix[1][2] * other.z,
                            matrix[2][0] * other.x + matrix[2][1] * other.y + matrix[2][2] * other.z);
    }

    @Override
    public double det() {
        return  matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }

    @Override
    public void transpose() {
        double[][] prev = matrix;
        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                matrix[m][n] = prev[n][m];
            }
        }
    }

    @Override
    public Matrix3d transposed() {
        double[][] nMatrix = matrix;
        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix[m][n] = matrix[n][m];
            }
        }
        return new Matrix3d(nMatrix);
    }

    @Override
    public void invert() {
        Matrix3d nMatrix = new Matrix3d(matrix);
        double det = det();

        //find cofactor matrix
        int coefficient = 1;
        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix.matrix[m][n] = coefficient * quick2dDet(minor(m, n));
                coefficient *= -1;
            }
        }

        //transpose to get adjugate matrix
        nMatrix.transpose();

        //multiply by 1 over the original matrix's determinant
        nMatrix.times(1/det);

        //replace this matrix double arr with nMatrix's
        matrix = nMatrix.matrix;
    }

    @Override
    public Matrix3d inverted() {
        Matrix3d nMatrix = new Matrix3d(matrix);
        double det = det();

        //find cofactor matrix
        int coefficient = 1;
        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix.matrix[m][n] = coefficient * quick2dDet(minor(m, n));
                coefficient *= -1;
            }
        }

        //transpose to get the adjugate matrix
        nMatrix.transpose();

        //multiply by 1 over the original matrix's determinant to get the inverse and return it
        return nMatrix.times(1/det);
    }

    private double quick2dDet(double[] arr){
        return arr[0] * arr[3] + arr[1] * arr[2];
    }

    private double[] minor(int m, int n){
        double[] output = new double[4];
        int i = 0;
        for (int y = 0; y < 3; y++){
            for (int x = 0; x < 3; x++){
                if (y != m && x != n){
                    output[i] = matrix[y][x];
                    i++;
                }
            }
        }

        return output;
    }

    //methods for preset matrices

    public static Matrix3d makeAffineTranslation(double translationX, double translationY){
        return new Matrix3d(new double[][]{
                {1, 0, translationX},
                {0, 1, translationY},
                {0, 0, 1}
        });
    }

    public static Matrix3d makeAffineTranslation(Vector2d translation){
        return new Matrix3d(new double[][]{
                {1, 0, translation.x},
                {0, 1, translation.y},
                {0, 0, 1}
        });
    }

    public static Matrix3d makeAffineRotation(double angle){
        double cosine = Math.cos(angle);
        double sine = Math.sin(angle);

        return new Matrix3d(new double[][]{
                {cosine, -sine, 0},
                {sine, cosine, 0},
                {0, 0, 1}
        });
    }

    public static Matrix3d makeAffineScale(double scale){
        return new Matrix3d(new double[][]{
                {scale, 0, 0},
                {0, scale, 0},
                {0, 0, 1}
        });
    }

    public static Matrix3d makeAffineScale(double xScale, double yScale){
        return new Matrix3d(new double[][]{
                {xScale, 0, 0},
                {0, yScale, 0},
                {0, 0, 1}
        });
    }

    public static Matrix3d makeAffineColumnSpace(Vector2d xAxis, Vector2d yAxis){
        return new Matrix3d(new double[][]{
                {xAxis.x, yAxis.x, 0},
                {xAxis.y, yAxis.y, 0},
                {0, 0, 1}
        });
    }

}
