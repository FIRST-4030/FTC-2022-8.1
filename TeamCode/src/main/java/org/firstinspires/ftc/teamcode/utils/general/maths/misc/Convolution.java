package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

public class Convolution {

    public interface OneInputFunction<T1, R1>{
        R1 eval(T1 a);
    }
    public interface TwoInputFunction<T1, T2, R1>{
        R1 eval(T1 a, T2 b);
    }

    public static final TwoInputFunction<double[], double[], double[]> complexSum = (a, b) -> Complex.addAsComplex(a[0], a[1], b[0], b[1]);
    public static final TwoInputFunction<double[], double[], double[]> complexMul = (a, b) -> Complex.multiplyAsComplex(a[0], a[1], b[0], b[1]);

    /**
     * Using the black magic of FFTs, we can get the time complexity of a normal convolution from O(N^2) to O(N * ln(N))
     * @param arr
     * @param convolution
     * @return
     */
    public static double[] fftConvolve(double[] arr, double[] convolution){
        int outputLen = arr.length + convolution.length - 1;
        int r = outputLen % 2;

        //apply FFT to both inputs after zero padding to get them to final length
        double[][] fftA = FFT.fftI(
                arr1dToArray2d(
                        FFT.padToNearestBase2(
                                FFT.zeroPadTo(arr, outputLen + r)
                        )
                )
        );
        double[][] fftB = FFT.fftI(
                arr1dToArray2d(
                        FFT.padToNearestBase2(
                                FFT.zeroPadTo(convolution, outputLen + r)
                        )
                )
        );

        double[][] ifftR = FFT.ifftI(mul(fftA, fftB, complexMul));

        return FFT.real(FFT.truncateData(ifftR, 0, outputLen));
    }

    public static double[][] arr1dToArray2d(double[] arr){
        double[][] output = new double[arr.length][2]; //real and imaginary per slot
        for (int i = 0; i < arr.length; i++){
            output[i][0] = arr[i];
            output[i][1] = 0d;
        }
        return output;
    }

    public static double[] add(double[] a, double[] b){
        if (a.length != b.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + a.length + " B len: " + b.length);
        double[] output = new double[a.length];
        for (int i = 0; i < output.length; i++){
            output[i] = a[i] + b[i];
        }
        return output;
    }

    public static double[] add(double[] a, double[] b, TwoInputFunction<Double, Double, Double> additionBehavior){
        if (a.length != b.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + a.length + " B len: " + b.length);
        double[] output = new double[a.length];
        for (int i = 0; i < output.length; i++){
            output[i] = additionBehavior.eval(a[i], b[i]);
        }
        return output;
    }

    public static double[][] add(double[][] complexA, double[][] complexB, TwoInputFunction<double[], double[], double[]> additionBehavior){
        if (complexA.length != complexB.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + complexA.length + " B len: " + complexB.length);
        double[][] output = new double[complexA.length][2];
        for (int i = 0; i < output.length; i++){
            output[i] = additionBehavior.eval(complexA[i], complexB[i]);
        }
        return output;
    }

    public static double[] mul(double[] a, double[] b){
        if (a.length != b.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + a.length + " B len: " + b.length);
        double[] output = new double[a.length];
        for (int i = 0; i < output.length; i++){
            output[i] = a[i] * b[i];
        }
        return output;
    }

    public static double[][] mul(double[][] a, double[][] b){
        if (a.length != b.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + a.length + " B len: " + b.length);
        double[][] output = new double[a.length][2];
        for (int i = 0; i < output.length; i++){
            output[i][0] = a[i][0] * b[i][0];
            output[i][1] = a[i][1] * b[i][1];
        }
        return output;
    }

    public static double[] mul(double[] a, double[] b, TwoInputFunction<Double, Double, Double> multiplicationBehavior){
        if (a.length != b.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + a.length + " B len: " + b.length);
        double[] output = new double[a.length];
        for (int i = 0; i < output.length; i++){
            output[i] = multiplicationBehavior.eval(a[i], b[i]);
        }
        return output;
    }

    public static double[][] mul(double[][] complexA, double[][] complexB, TwoInputFunction<double[], double[], double[]> multiplicationBehavior){
        if (complexA.length != complexB.length) throw new IndexOutOfBoundsException("Argument A or B (or both) are not equal in length for this operation! A len: " + complexA.length + " B len: " + complexB.length);
        double[][] output = new double[complexA.length][2];
        for (int i = 0; i < output.length; i++){
            output[i] = multiplicationBehavior.eval(complexA[i], complexB[i]);
        }
        return output;
    }
}
