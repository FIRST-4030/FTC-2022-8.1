package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.controllers;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.misc.ConvergenceFunction;

public class NormalizedController {
    private ConvergenceFunction normalizer;

    private double maxExpectedErrorValue;

    private double iErrorSum, iErrorSumRadius;
    private double lastTime, currTime;
    private double kP, kI, kD;

    private double savedTarget, savedCurrent;

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

    public NormalizedController setMaxExpectedError(double nMaxExpectedError){
        this.maxExpectedErrorValue = nMaxExpectedError;
        return this;
    }

    public void reset(){
        this.iErrorSum = 0;
        this.savedTarget = 0;
        this.savedCurrent = 0;
    }

    /**
     * "It just works." - Todd Howard
     * @return
     */
    public double seek(){
        return seek(savedTarget, savedCurrent);
    }

    /**
     * "It just works." - Todd Howard
     * @param current
     * @return
     */
    public double seek(double current){
        return seek(savedTarget, current);
    }


    /**
     * "It just works." - Todd Howard
     * @param target
     * @param current
     * @return
     */
    public double seek(double target, double current){
        double p, d;
        savedTarget = target;
        savedCurrent = current;

        currTime = Math.min(Math.max((target - current) / maxExpectedErrorValue, -1), 1); //Clamp normalized values to [-1, 1]

        p = normalizer.eval(currTime); //feedforward modelled behavior
        d = -normalizer.derivative(currTime); //feedforward modelled derivative

        //Traditional I term application used a left Riemann sum on the right and right Riemann sum on the left,
        //resulting in an underestimate of the actual integral all the time
        if (currTime < iErrorSumRadius) iErrorSum += normalizer.integral(Math.min(currTime, lastTime), Math.max(currTime, lastTime));

        lastTime = currTime;

        return kP * p + kI * iErrorSum + kD * d; //Apply feedforward as a typical PID sum
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
