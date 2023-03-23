package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import java.util.Locale;

public class Complex {

    private double re, im;

    public Complex(){
        this(0, 0);
    }

    public Complex(double r, double i){
        this.re = r;
        this.im = i;
    }

    public Complex plus(Complex b) {
        return new Complex(this.re + b.re, this.im + b.im);
    }

    public static double[] addAsComplex(double realA, double imgA, double realB, double imgB){
        return new double[]{realA + realB, imgA + imgB};
    }

    public Complex minus(Complex b) {
        return new Complex(this.re - b.re, this.im - b.im);
    }

    public static double[] subtractAsComplex(double realA, double imgA, double realB, double imgB){
        return new double[]{realA - realB, imgA - imgB};
    }

    public Complex times(Complex b) {
        return new Complex(this.re * b.re - this.im * b.im,
                this.re * b.im + this.im * b.re);
    }

    public static double[] multiplyAsComplex(double realA, double imgA, double realB, double imgB){
        return new double[]{realA * realB - imgA * imgB, realA * imgB + imgA * realB};
    }

    public void conjugate(){
        this.im *= -1;
    }
    public Complex getAsConjugate(){
        return new Complex(re, -im);
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "(%f,%f)", re, im);
    }
}
