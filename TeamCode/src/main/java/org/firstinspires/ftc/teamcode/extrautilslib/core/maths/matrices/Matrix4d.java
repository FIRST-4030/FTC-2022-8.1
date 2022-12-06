package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;

public class Matrix4d implements EULMatrix<Matrix4d, Vector4d>{

    public double[][] matrix;

    public Matrix4d(){
        matrix = new double[][]{{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    }

    public Matrix4d(double[] firstRow, double[] secondRow, double[] thirdRow, double[] fourthRow){
        if (firstRow.length == 4 && secondRow.length == 4 && thirdRow.length == 4 && fourthRow.length == 4) {
            matrix = new double[][]{firstRow, secondRow, thirdRow, fourthRow};
        } else {
            matrix = new double[][]{{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        }
    }

    public Matrix4d(double[][] arr){
        if (arr.length == 4 && arr[0].length == 4) {
            matrix = arr;
        } else {
            matrix = new double[][]{{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        }
    }

    @Override
    public String toString(){
        return"{{" + matrix[0][0] + ", " + matrix[0][1] + ", " + matrix[0][2] + ", " + matrix[0][3] + "}," +
              "{" + matrix[1][0] + ", " + matrix[1][1] + ", " + matrix[1][2] + ", " + matrix[1][3] + "}," +
              "{" + matrix[2][0] + ", " + matrix[2][1] + ", " + matrix[2][2] + ", " + matrix[2][3] + "}," +
              "{" + matrix[3][0] + ", " + matrix[3][1] + ", " + matrix[3][2] + ", " + matrix[3][3] + "}}";
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
    public Matrix4d plus(Matrix4d other) {
        return new Matrix4d(new double[][]{
                {matrix[0][0] + other.matrix[0][0], matrix[0][1] + other.matrix[0][1], matrix[0][2] + other.matrix[0][2], matrix[0][3] + other.matrix[0][3]},
                {matrix[1][0] + other.matrix[1][0], matrix[1][1] + other.matrix[1][1], matrix[1][2] + other.matrix[1][2], matrix[1][3] + other.matrix[1][3]},
                {matrix[2][0] + other.matrix[2][0], matrix[2][1] + other.matrix[2][1], matrix[2][2] + other.matrix[2][2], matrix[2][3] + other.matrix[2][3]},
                {matrix[3][0] + other.matrix[3][0], matrix[3][1] + other.matrix[3][1], matrix[3][2] + other.matrix[3][2], matrix[3][3] + other.matrix[3][3]}
        });
    }

    @Override
    public Matrix4d minus(Matrix4d other) {
        Matrix4d negativeMatrix = unaryMinus();
        return plus(negativeMatrix);
    }

    @Override
    public Matrix4d unaryMinus() {
        return new Matrix4d(new double[][]{
                {-matrix[0][0], -matrix[0][1], -matrix[0][2], -matrix[0][3]},
                {-matrix[1][0], -matrix[1][1], -matrix[1][2], -matrix[1][3]},
                {-matrix[2][0], -matrix[2][1], -matrix[2][2], -matrix[2][3]},
                {-matrix[3][0], -matrix[3][1], -matrix[3][2], -matrix[3][3]}
        });
    }

    @Override
    public Matrix4d times(double other) {
        return new Matrix4d(new double[][]{
                {matrix[0][0] * other, matrix[0][1] * other, matrix[0][2] * other, matrix[0][3] * other},
                {matrix[1][0] * other, matrix[1][1] * other, matrix[1][2] * other, matrix[1][3] * other},
                {matrix[2][0] * other, matrix[2][1] * other, matrix[2][2] * other, matrix[2][3] * other},
                {matrix[3][0] * other, matrix[3][1] * other, matrix[3][2] * other, matrix[3][3] * other}
        });
    }

    @Override
    public Matrix4d times(Matrix4d other) {

        double[][] nMatrix = new double[4][4];

        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix[m][n] = matrix[m][0] * other.matrix[0][n] + matrix[m][1] * other.matrix[1][n] + matrix[m][2] * other.matrix[2][n] + matrix[m][3] * other.matrix[3][n];
            }
        }

        return new Matrix4d(nMatrix);
    }

    @Override
    public Vector4d times(Vector4d other) {
        return new Vector4d(
                matrix[0][0] * other.x + matrix[0][1] * other.y + matrix[0][2] * other.z + matrix[0][3] * other.w,
                matrix[1][0] * other.x + matrix[1][1] * other.y + matrix[1][2] * other.z + matrix[1][3] * other.w,
                matrix[2][0] * other.x + matrix[2][1] * other.y + matrix[2][2] * other.z + matrix[2][3] * other.w,
                matrix[3][0] * other.x + matrix[3][1] * other.y + matrix[3][2] * other.z + matrix[3][3] * other.w
        );
    }

    @Override
    public double det() {
        return matrix[0][0] * quick3dDet(minor(0,0)) - matrix[0][1] * quick3dDet(minor(0, 1)) + matrix[0][2] * quick3dDet(minor(0, 2)) - matrix[0][3] * quick3dDet(minor(0, 3));
    }

    @Override
    public void transpose() {
        double[][] prev = matrix;
        for (int m = 0; m < 4; m++){
            for (int n = 0; n < 4; n++){
                matrix[m][n] = prev[n][m];
            }
        }
    }

    @Override
    public Matrix4d transposed() {
        double[][] nMatrix = matrix;
        for (int m = 0; m < 3; m++){
            for (int n = 0; n < 3; n++){
                nMatrix[m][n] = matrix[n][m];
            }
        }
        return new Matrix4d(nMatrix);
    }

    @Override
    public void invert() {
        Matrix4d nMatrix = new Matrix4d(matrix);
        double det = det();

        //find cofactor matrix
        int coefficient = 1;
        for (int m = 0; m < 4; m++){
            for (int n = 0; n < 4; n++){
                nMatrix.matrix[m][n] = coefficient * quick3dDet(minor(m, n));
                coefficient *= -1; //makes the elements follow this array of coefficients(according to their element): {{-, +, -, +}, {-, +, -, +}, {-, +, -, +}, {-, +, -, +}}
            }
            coefficient *= -1; //makes {{-, +, -, +}, {-, +, -, +}, {-, +, -, +}, {-, +, -, +}} into {{-, +, -, +}, {+, -, +, -}, {-, +, -, +}, {+, -, +, -}} as it is required for finding the cofactor matrix
        }

        //transpose to get adjugate matrix
        nMatrix.transpose();

        //multiply by 1 over the original matrix's determinant
        nMatrix.times(1/det);

        //replace this matrix double arr with nMatrix's
        matrix = nMatrix.matrix;
    }

    @Override
    public Matrix4d inverted() {
        Matrix4d nMatrix = new Matrix4d(matrix);
        double det = det();

        //find cofactor matrix
        int coefficient = 1;
        for (int m = 0; m < 4; m++){
            for (int n = 0; n < 4; n++){
                nMatrix.matrix[m][n] = coefficient * quick3dDet(minor(m, n));
                coefficient *= -1; //makes the elements follow this array of coefficients(according to their element): {{-, +, -, +}, {-, +, -, +}, {-, +, -, +}, {-, +, -, +}}
            }
            coefficient *= -1; //makes {{-, +, -, +}, {-, +, -, +}, {-, +, -, +}, {-, +, -, +}} into {{-, +, -, +}, {+, -, +, -}, {-, +, -, +}, {+, -, +, -}} as it is required for finding the cofactor matrix
        }

        //transpose to get the adjugate matrix
        nMatrix.transpose();

        //multiply by 1 over the original matrix's determinant to get the inverse and return it
        return nMatrix.times(1/det);
    }

    private double quick3dDet(double[] arr){
        return arr[0] * (arr[4] * arr[8] - arr[5] * arr[7]) - arr[1] * (arr[3] * arr[8] - arr[5] * arr[6]) + arr[3] * (arr[3] * arr[7] - arr[4] * arr[6]);
    }

    private double[] minor(int m, int n){
        double[] output = new double[9];
        int i = 0;
        for (int y = 0; y < 4; y++){
            for (int x = 0; x < 4; x++){
                if (y != m && x != n){
                    output[i] = matrix[y][x];
                    i++;
                }
            }
        }

        return output;
    }

    //methods for preset matrices

    public static Matrix4d makeAffineTranslation(double translationX, double translationY, double translationZ){
        return new Matrix4d(new double[][]{
                {1, 0, 0, translationX},
                {0, 1, 0, translationY},
                {0, 0, 1, translationZ},
                {0, 0, 0, 1}
        });
    }

    public static Matrix4d makeAffineTranslation(Vector3d translation){
        return new Matrix4d(new double[][]{
                {1, 0, 0, translation.x},
                {0, 1, 0, translation.y},
                {0, 0, 1, translation.z},
                {0, 0, 0, 1}
        });
    }

    public static Matrix4d makeAffineRotation(EULMathEx.Axis revAxis, double angle){
        double cosine = Math.cos(angle);
        double sine = Math.sin(angle);
        Matrix4d output = new Matrix4d();

        switch (revAxis){
            case AXIS_X:
                output.matrix = new double[][]{
                        {1, 0, 0, 0},
                        {0, cosine, -sine, 0},
                        {0, sine, cosine, 0},
                        {0, 0, 0, 1}
                };
                break;
            case AXIS_Y:
                output.matrix = new double[][]{
                        {cosine, 0, sine, 0},
                        {0, 1, 0, 0},
                        {-sine, 0, cosine, 0},
                        {0, 0, 0, 1}
                };
                break;
            case AXIS_Z:
                output.matrix = new double[][]{
                        {cosine, -sine, 0, 0},
                        {sine, cosine, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1}
                };
                break;
        }

        return output;
    }

    public static Matrix4d makeAffineGenRotation(double angleX, double angleY, double angleZ){
        double sineAlpha = Math.sin(angleX), cosineAlpha = Math.cos(angleX);
        double sineBeta = Math.sin(angleY), cosineBeta = Math.cos(angleY);
        double sineGamma = Math.sin(angleZ), cosineGamma = Math.cos(angleZ);

        Matrix4d output = new Matrix4d(new double[][]{
                {cosineBeta * cosineGamma, sineAlpha * sineBeta * cosineGamma - cosineAlpha * sineGamma, cosineAlpha * sineBeta * cosineGamma + sineAlpha * sineGamma, 0},
                {  cosineBeta * sineGamma, sineAlpha * sineBeta * sineGamma + cosineAlpha * cosineGamma, cosineAlpha * sineBeta * sineGamma - sineAlpha * cosineGamma, 0},
                {               -sineBeta,                                       sineAlpha * cosineBeta,                                     cosineAlpha * cosineBeta, 0},
                {                       0,                                                            0,                                                            0, 1}
        });

        return output;
    }

    public static Matrix4d makeAffineScale(double scale){
        return new Matrix4d(new double[][]{
                {scale, 0, 0, 0},
                {0, scale, 0, 0},
                {0, 0, scale, 0},
                {0, 0, 0, 1}
        });
    }

    public static Matrix4d makeAffineScale(double xScale, double yScale, double zScale){
        return new Matrix4d(new double[][]{
                {xScale, 0, 0, 0},
                {0, yScale, 0, 0},
                {0, 0, zScale, 0},
                {0, 0, 0, 1}
        });
    }

    public static Matrix4d makeAffineColumnSpace(Vector3d xAxis, Vector3d yAxis, Vector3d zAxis){
        return new Matrix4d(new double[][]{
                {xAxis.x, yAxis.x, zAxis.x, 0},
                {xAxis.y, yAxis.y, zAxis.y, 0},
                {xAxis.z, yAxis.z, zAxis.z, 0},
                {0, 0, 0, 1}
        });
    }
}
