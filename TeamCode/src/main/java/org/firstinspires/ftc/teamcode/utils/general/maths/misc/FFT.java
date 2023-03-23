package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

public class FFT {

    private static final double log2 = Math.log(2);

    private static int bitReverse(int n, int bits) {
        int reversedN = n;
        int count = bits - 1;

        n >>= 1;
        while (n > 0) {
            reversedN = (reversedN << 1) | (n & 1);
            count--;
            n >>= 1;
        }

        return ((reversedN << count) & ((1 << bits) - 1));
    }

    /**
     * This is a modified version of <a href="https://rosettacode.org/wiki/Fast_Fourier_transform#Java">original FFT Java implementation</a>
     * <br>This method stands for Fast Fourier Transform Iterative which is less memory intensive than the recursive version
     * <br>The limitation of this method is still having outputs that are the length of a base 2 number (4, 8, 16, 32, 64...etc)
     * @param real
     * @param imaginary
     * @return
     */
    public static double[][] fftI(double[] real, double[] imaginary) {
        if (real.length != imaginary.length) throw new IllegalArgumentException("FFT Class: Real and Imaginary lists don't match!");

        double[][] output = new double[real.length][2];

        for (int i = 0; i < output.length; i++) {
            output[i][0] = real[i];
            output[i][1] = imaginary[i];
        }

        int bits = findNearestCeilBase2(real.length);
        for (int j = 1; j < real.length / 2; j++) {

            int swapPos = bitReverse(j, bits);
            output[j][0] = real[swapPos];
            output[j][1] = imaginary[swapPos];
            output[swapPos][0] = real[j];
            output[swapPos][1] = imaginary[j];
        }

        for (int N = 2; N <= real.length; N <<= 1) {
            for (int i = 0; i < real.length; i += N) {
                for (int k = 0; k < N / 2; k++) {

                    int evenIndex = i + k;
                    int oddIndex = i + k + (N / 2);
                    double evenRe = output[evenIndex][0];
                    double evenIm = output[evenIndex][1];


                    double oddRe = output[oddIndex][0];
                    double oddIm = output[oddIndex][1];

                    // constructing expI
                    double x = -2d * Math.PI * k / (double) N;
                    double expXRe = Math.cos(x);
                    double expXIm = Math.sin(x);

                    // now multiplying exp with odd
                    double wmRe = expXRe * oddRe - expXIm * oddIm;
                    double wmIm = expXRe * oddIm + expXIm * oddRe;

                    output[evenIndex][0] = evenRe + wmRe;
                    output[evenIndex][1] = evenIm + wmIm;

                    output[oddIndex][0] = evenRe - wmRe;
                    output[oddIndex][1] = evenIm - wmIm;
                }
            }
        }
        return output;
    }

    /**
     * This is an overload version of fftI where it decomposes an input of n length with a list of two elements in each slot into a real and imaginary input
     * @param input
     * @return
     */
    public static double[][] fftI(double[][] input){
        double[] real = new double[input.length];
        double[] imaginary = new double[input.length];

        for (int i = 0; i < input.length; i++) {
            real[i] = input[i][0];
            imaginary[i] = input[i][1];
        }

        return fftI(real, imaginary);
    }

    static void fft(Complex[] buffer) {

        int bits = findNearestCeilBase2(buffer.length);
        for (int j = 1; j < buffer.length / 2; j++) {

            int swapPos = bitReverse(j, bits);
            Complex temp = buffer[j];
            buffer[j] = buffer[swapPos];
            buffer[swapPos] = temp;
        }

        for (int N = 2; N <= buffer.length; N <<= 1) {
            for (int i = 0; i < buffer.length; i += N) {
                for (int k = 0; k < N / 2; k++) {

                    int evenIndex = i + k;
                    int oddIndex = i + k + (N / 2);
                    Complex even = buffer[evenIndex];
                    Complex odd = buffer[oddIndex];

                    double term = (-2 * Math.PI * k) / (double) N;
                    Complex exp = (new Complex(Math.cos(term), Math.sin(term)).times(odd));

                    buffer[evenIndex] = even.plus(exp);
                    buffer[oddIndex] = even.plus(exp);
                }
            }
        }
    }

    /**
     * This is a modified version FFT to get an IFFT using the #3 method described <a href="https://www.dsprelated.com/showarticle/800.php">here</a>
     * <br>This method stands for Inverse Fast Fourier Transform Iterative which is less memory intensive than the recursive version
     * <br>The limitation of this method is still having outputs that are the length of a base 2 number (4, 8, 16, 32, 64...etc)
     * @param real
     * @param imaginary
     * @return
     */
    public static double[][] ifftI(double[] real, double[] imaginary) {
        if (real.length != imaginary.length) throw new IllegalArgumentException("FFT Class: Real and Imaginary lists don't match!");

        double[][] output = new double[real.length][2];
        double iN = (1d / output.length);

        for (int i = 0; i < output.length; i++) {
            output[i][1] *= -1;
        }

        output = fftI(output);

        for (int i = 0; i < output.length; i++) {
            output[i][0] *= iN;
            output[i][1] *= -1 * iN;
        }

        return output;
    }

    /**
     * This is an overload version of ifftI where it decomposes an input of n length with a list of two elements in each slot into a real and imaginary input
     * @param input
     * @return
     */
    public static double[][] ifftI(double[][] input){
        double[] real = new double[input.length];
        double[] imaginary = new double[input.length];

        for (int i = 0; i < input.length; i++) {
            real[i] = input[i][0];
            imaginary[i] = input[i][1];
        }

        return ifftI(real, imaginary);
    }

    public static double[] addZeroPad(double[] arr, int toAdd){
        double[] output = new double[arr.length + toAdd];
        System.arraycopy(arr, 0, output, 0, arr.length);
        return output;
    }

    public static double[] zeroPadTo(double[] arr, int finalLen){
        double[] output = new double[finalLen];
        System.arraycopy(arr, 0, output, 0, Math.min(arr.length, output.length));
        return output;
    }

    public static int findNearestCeilBase2(int num){
        if (num <= 1) return 0;
        return (int) Math.ceil(Math.log(num) / log2);
    }

    public static double[] padToNearestBase2(double[] arr){
        return zeroPadTo(arr, (int) Math.pow(2, findNearestCeilBase2(arr.length)));
    }

    public static double[] truncateData(double[] data, int start, int end){
        double[] output = new double[end - start];
        System.arraycopy(data, start, output, 0, end - start);
        return output;
    }

    public static double[][] truncateData(double[][] data, int start, int end){
        double[][] output = new double[end - start][2];
        System.arraycopy(data, start, output, 0, end - start);
        return output;
    }

    public static double[] real(double[][] data){
        double[] output = new double[data.length];
        for (int i = 0; i < output.length; i++){
            output[i] = data[i][0];
        }
        return output;
    }
}
