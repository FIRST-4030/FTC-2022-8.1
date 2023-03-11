package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.misc;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.misc.IndentedToString;

public class ConvergenceFunction implements IndentedToString {

    private double k;

    public ConvergenceFunction(double bias){
        double clampedBias = clamp(0, 1, bias);
        this.k = (1 + clampedBias) * (1 + clampedBias) * (1 + clampedBias);
    }
    private double clamp(double min, double max, double val){
        return Math.min(max, Math.max(min, val));
    }

    public double eval(double x){
        return (Math.abs(x) * k) / (Math.abs(x)*k - Math.abs(x) + 1) * Math.signum(x);
    }

    public double derivative(double x){
        return (Math.abs(x) * Math.abs(x) * k) / (((k-1) * Math.abs(x) + 1) * ((k-1) * Math.abs(x) + 1));
    }

    public double antiderivative(double x){
        if (k != 0.0) {
            return k * ((k - 1) * Math.abs(x) - Math.log((k - 1) * Math.abs(x) + 1)) / ((k - 1) * (k - 1));
        } else {
            return 0.5 * (x * x);
        }
    }

    public double integral(double a, double b){
        return antiderivative(b) - antiderivative(a);
    }

    @Override
    public String toString() {
        return "ConvergenceFunction {" + "\n\t" +
                "bias = " + k + "\n" +
                "}";
    }

    @Override
    public String toString(int indentation) {
        StringBuilder tabs = new StringBuilder("\n");
        for (int i = 0; i < indentation; i++) tabs.append("\t");
        return tabs + "ConvergenceFunction {" + "\n\t" + tabs +
                "bias = " + k + "\n" + tabs +
                "}";
    }
}
