package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.controllers;

public class GeneralPIDController {

    private double lastTime, currTime, deltaTime; //seconds
    private double lastError, currError;
    private double iErrorAccumulation, accumulationRadius;
    private double kP, kI, kD;

    public GeneralPIDController(double kP, double kI, double kD, double iErrorAccumulationRadius){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        iErrorAccumulation = 0d;
        accumulationRadius = iErrorAccumulationRadius;

        lastError = 0d;
        currError = 0d;

        lastTime = 0d;
        currTime = 0d;
        deltaTime = 0d;
    }

    public double seek(double target, double current){
        currError = target - current;
        currTime = System.currentTimeMillis() / 1000d;
        deltaTime = currTime - lastTime;

        double p = currError;
        if (Math.abs(currError) < accumulationRadius) iErrorAccumulation += currError * deltaTime;
        double d = 0;
        if (lastTime != 0) d = (currError - lastError) / (deltaTime);

        lastTime = currTime;
        lastError = currError;
        return kP * p + kI * iErrorAccumulation + kD * d;
    }

    public void reset(){
        iErrorAccumulation = 0;
    }

    public GeneralPIDController setKp(double nKp){
        this.kP = nKp;
        return this;
    }

    public double getKp(){
        return kP;
    }

    public GeneralPIDController setKi(double nKi){
        this.kI = nKi;
        return this;
    }

    public double getKi(){
        return kI;
    }

    public GeneralPIDController setKd(double nKd){
        this.kD = nKd;
        return this;
    }

    public double getKd(){
        return kD;
    }

    public GeneralPIDController setAccumulationRadius(double nAccumulationRadius){
        this.accumulationRadius = nAccumulationRadius;
        return this;
    }

    public double getAccumulationRadius(){
        return this.accumulationRadius;
    }
}
