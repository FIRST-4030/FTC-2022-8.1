package org.firstinspires.ftc.teamcode.utils.general.misc.pidcontroller.controllers;

import org.firstinspires.ftc.teamcode.utils.general.misc.pidcontroller.misc.ConvergenceFunction;

public class NormalizedController {
    private ConvergenceFunction normalizer;

    private double maxExpectedErrorValue;

    private double iErrorSum, iErrorSumRadius;
    private double lastTime, currTime;
    private double kP, kI, kD;

    public NormalizedController(double kP, double kI, double kD){
        this(kP, kI, kD, 0.2, 0.7);
    }

    public NormalizedController(double kP, double kI, double kD, double iErrorSumRadius, double normalizerBias){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.iErrorSum = 0d;
        this.iErrorSumRadius = iErrorSumRadius;

        this.lastTime = 0; //simulated "time"
        this.currTime = 0; //simulated "time"

        this.normalizer = new ConvergenceFunction(normalizerBias);
    }

    private double clamp(double min, double max, double value){
        return Math.min(max, Math.max(value, min));
    }

    public NormalizedController setMaxExpectedError(double nMaxExpectedError){
        this.maxExpectedErrorValue = nMaxExpectedError;
        return this;
    }

    public void reset(){
        this.iErrorSum = 0;
    }

    /**
     * "It just works." - Todd Howard
     * @param target
     * @param current
     * @return
     */
    public double seek(double target, double current){
        double p, d;

        currTime = clamp(-1, 1, (target - current) / maxExpectedErrorValue);

        p = normalizer.eval(currTime);
        d = -normalizer.derivative(currTime);

        if (currTime < iErrorSumRadius) iErrorSum += p * (currTime - lastTime);

        return kP * p + kI * iErrorSum + kD * d;
    }

    public double getKp() {
        return kP;
    }

    public void setKp(double kP) {
        this.kP = kP;
    }

    public double getKi() {
        return kI;
    }

    public void setKi(double kI) {
        this.kI = kI;
    }

    public double getKd() {
        return kD;
    }

    public void setKd(double kD) {
        this.kD = kD;
    }

    public double getErrorSumRadius() {
        return iErrorSumRadius;
    }

    public void setErrorSumRadius(double iErrorSumRadius) {
        this.iErrorSumRadius = iErrorSumRadius;
    }

    public double getMaxExpectedErrorValue() {
        return maxExpectedErrorValue;
    }

    public void setMaxExpectedErrorValue(double maxExpectedErrorValue) {
        this.maxExpectedErrorValue = maxExpectedErrorValue;
    }
}
