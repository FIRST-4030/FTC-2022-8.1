package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

public class FFT {

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
     * @param real
     * @param imaginary
     * @return
     */
    public static double[][] fftI(double[] real, double[] imaginary) {
        if (real.length != imaginary.length) throw new IllegalArgumentException("FFT Class: Real and Imaginary lists don't match!");

        double[][] output = new double[real.length][2];

        int bits = (int) (Math.log(real.length) / Math.log(2));
        for (int j = 1; j < real.length / 2; j++) {

            int swapPos = bitReverse(j, bits);
            double tempR = real[j];
            double tempI = imaginary[j];
            output[j][0] = real[swapPos];
            output[j][1] = imaginary[swapPos];
            output[swapPos][0] = tempR;
            output[swapPos][1] = tempI;
        }

        for (int N = 2; N <= real.length; N <<= 1) {
            for (int i = 0; i < real.length; i += N) {
                for (int k = 0; k < N / 2; k++) {

                    int evenIndex = i + k;
                    int oddIndex = i + k + (N / 2);

                    double evenReal = output[evenIndex][0];
                    double evenImaginary = output[evenIndex][1];

                    double oddReal = output[oddIndex][0];
                    double oddImaginary = output[oddIndex][1];

                    double term = (-2 * Math.PI * k) / N;
                    double[] exp = Complex.multiplyAsComplex(Math.cos(term), Math.sin(term), oddReal, oddImaginary);

                    output[evenIndex][0] = evenReal + exp[0];
                    output[evenIndex][1] = evenImaginary + exp[1];

                    output[oddIndex][0] = evenReal - exp[0];
                    output[oddIndex][1] = evenImaginary - exp[1];
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

    /**
     * This is a modified version FFT to get an IFFT using the #3 method described <a href="https://www.dsprelated.com/showarticle/800.php">here</a>
     * <br>This method stands for Inverse Fast Fourier Transform Iterative which is less memory intensive than the recursive version
     * @param real
     * @param imaginary
     * @return
     */
    public static double[][] ifftI(double[] real, double[] imaginary) {
        if (real.length != imaginary.length) throw new IllegalArgumentException("FFT Class: Real and Imaginary lists don't match!");

        double[][] output = new double[real.length][2];

        double[] tempArr = new double[real.length];
        System.arraycopy(real, 0, tempArr, 0, real.length);
        real = imaginary;
        imaginary = tempArr;

        int bits = (int) (Math.log(real.length) / Math.log(2));
        for (int j = 1; j < real.length / 2; j++) {

            int swapPos = bitReverse(j, bits);
            double tempR = real[j];
            double tempI = imaginary[j];
            output[j][0] = real[swapPos];
            output[j][1] = imaginary[swapPos];
            output[swapPos][0] = tempR;
            output[swapPos][1] = tempI;
        }

        for (int N = 2; N <= real.length; N <<= 1) {
            for (int i = 0; i < real.length; i += N) {
                for (int k = 0; k < N / 2; k++) {

                    int evenIndex = i + k;
                    int oddIndex = i + k + (N / 2);

                    double evenReal = output[evenIndex][0];
                    double evenImaginary = output[evenIndex][1];

                    double oddReal = output[oddIndex][0];
                    double oddImaginary = output[oddIndex][1];

                    double term = (2 * Math.PI * k) / N; //I believe this is the inverse term
                    double[] exp = Complex.multiplyAsComplex(Math.cos(term), Math.sin(term), oddReal, oddImaginary);

                    output[evenIndex][1] = evenReal + exp[0];
                    output[evenIndex][0] = evenImaginary + exp[1];

                    output[oddIndex][1] = evenReal - exp[0];
                    output[oddIndex][0] = evenImaginary - exp[1];
                }
            }
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

        return fftI(imaginary, real);
    }
}
