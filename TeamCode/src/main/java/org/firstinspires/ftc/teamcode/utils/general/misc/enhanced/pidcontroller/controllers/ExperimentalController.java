package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.controllers;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.misc.ConvergenceFunction;

public class ExperimentalController {

    private ConvergenceFunction feedforward;
    private double feedforwardReactionBand;

    //kP, kI, kD are PID (Proportional, Integral, Derivative) constants for the feedback
    //kF and kA are extra constants to deal with the feedforward and acceleration
    private double kP, kI, kD, kA, kF;
    private double integralContributionRadius;
    private double internalWeightedAvg;

    private double savedPIDIntegral;

    private double savedTarget, savedOutput;
    private double lastDelta, lastPIDVelocity, lastTime;
    private double currentTime;

    public ExperimentalController(double p, double i, double d, double a, double f, double integralContributionRadius,  double feedforwardInterpolationBias, double feedforwardReactionBand){
        //Instantiate coefficients
        this.kP = p; //PID Proportional
        this.kI = i; //PID Integral
        this.kD = d; //PID Derivative
        this.kA = a; //Acceleration Contribution
        this.kF = f; //Feedforward contribution [0, 1]
        this.integralContributionRadius = integralContributionRadius;
        this.internalWeightedAvg = 1d - kF; //Feedback contribution [0, 1]

        //Make feedforward algorithm
        this.feedforward = new ConvergenceFunction(feedforwardInterpolationBias); //Feedforward algorithm
        this.feedforwardReactionBand = feedforwardReactionBand; //Radius where feedforward should slow down


        //Instantiate saved/persisting variables
        this.savedPIDIntegral = 0;
        this.savedTarget = 0;
        this.savedOutput = 0;

        this.currentTime = 1; //rather not set to 0 as it WILL cause a division-by-zero exception

        this.lastDelta = 0;
        this.lastPIDVelocity = 0;
        this.lastTime = 0;
    }

    private double clamp(double min, double max, double val){
        return Math.max(min, Math.min(max, val));
    }

    public void reset(){
        this.savedPIDIntegral = 0;
        this.lastTime = this.currentTime;
        this.currentTime = System.currentTimeMillis() / 1000d;
    }

    public double getOutput(double target, double current){
        this.lastTime = this.currentTime; //replace last time with time recorded last loop
        this.currentTime = System.currentTimeMillis() / 1000d; //get the current loop's time and convert to seconds domain

        this.savedTarget = target; //to be used when there's no change in args in a later method

        double delta = target - current; //calc current error
        double deltaTime = (this.currentTime - this.lastTime); //calc time taken

        double pTerm = this.clamp(-1, 1, delta / feedforwardReactionBand); //calc proportional error (in respect to the [-1, 1] output)

        if (Math.abs(delta) <= integralContributionRadius) {
            this.savedPIDIntegral += feedforward.integral(Math.min(0, delta / feedforwardReactionBand), Math.max(0, delta / feedforwardReactionBand));
        }

        double dTerm = this.clamp(-1, 1, (delta - lastDelta) / deltaTime); //calculate the predicted velocity
        double aTerm = this.clamp(-1, 1, (dTerm - lastPIDVelocity) / deltaTime); //calculate the predicted acceleration

        double pida = pTerm * this.kP + savedPIDIntegral * this.kI + dTerm * (this.kD - kA * aTerm); //classic pid

        this.savedOutput = this.clamp(-1, 1, pida) * internalWeightedAvg + feedforward.eval(delta / feedforwardReactionBand) * kF; //calculate the contribution the feedforward and feedback make and adds them together

        this.lastPIDVelocity = dTerm; //replace last predicted velocity with current velocity

        return this.savedOutput;
    }

    public double getOutput(double current){
        return getOutput(savedTarget, current);
    }

    public double getOutput(){
        return savedOutput;
    }
}
