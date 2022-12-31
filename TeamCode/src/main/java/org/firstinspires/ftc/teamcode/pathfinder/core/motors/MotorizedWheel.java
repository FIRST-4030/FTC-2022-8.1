package org.firstinspires.ftc.teamcode.pathfinder.core.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
//TODO: Add documentation
public class MotorizedWheel {

    //This is the intermediate function that converts tick deltas into power
    private static class PowerModulator{
        private int reactionBand = 100;
        private double k = 0.7;
        private double integralFactor1 = (k - 1);
        private double integralFactor2 = integralFactor1 * integralFactor1;

        public void updateBias(double bias){
            if (bias != 1){
                k = (1 - bias) * (1 - bias) * (1 - bias);
                integralFactor1 = k - 1;
                integralFactor2 = integralFactor1 * integralFactor1;
            } else {
                k = 0.7;
            }
        }

        public void updateReactionBand(int band){
            if (band != 0){
                this.reactionBand = Math.abs(band);
            }
        }

        public double function(double scalar){
            return 1 - (scalar * k) / (k * scalar - scalar + 1);
        }

        public double derivative(double scalar) {
            return (-k / (((k - 1) * scalar + 1) * ((k - 1) * scalar + 1)));
        }

        public double integral(double lowerBound, double upperBound){
            return antiderivative(upperBound) - antiderivative(lowerBound);
        }

        public double antiderivative(double scalar){
            double output;
            if (k == 1){
                output = -0.5 * (scalar * scalar) + 1;
            } else {
                output = (k * Math.log(integralFactor1 * scalar + 1) + (1 - k) * scalar) / integralFactor2;
            }
            return output;
        }
    }

    private DcMotorEx motor;
    private PowerModulator powerModulator;
    private double powerLimit = 1;
    private int tickReverse = 1;
    private int targetTickTolerance = 3;
    private double storedIntegral = 0;

    private double kP = 1, kI = 0, kD = 0;

    private boolean hasReachedTarget = true;

    public MotorizedWheel(HardwareMap hardwareMap, String motorName){
        this.motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        powerModulator = new PowerModulator();
    }

    public void setPIDGain(double kP, double kI, double kD){
        this.kP = kP; //initial reaction (Immediate reaction)
        this.kI = kI; //accumulated past reactions (Fine tune/ Controls precision)
        this.kD = kD; //prediction of a reaction (Dampens immediate reaction)
        //Don't worry about the I term as if set to zero, the JIT will do it's optimization magic
    }

    public void setOptions(boolean reverseDir, boolean reverseTick, int targetTickTolerance, double powerLimit, int reactionBand, double functionalBias){
        this.motor.setDirection(reverseDir ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        this.tickReverse = reverseTick ? -1 : 1;
        this.targetTickTolerance = targetTickTolerance / reactionBand;
        this.powerLimit = EULMathEx.doubleClamp(0, 1, Math.abs(powerLimit));
        this.powerModulator.updateReactionBand(reactionBand);
        this.powerModulator.updateBias(functionalBias);
    }

    public void setPower(double power){
        this.motor.setPower(power);
    }

    public int getCurrentEncoderTicks(){
        return this.motor.getCurrentPosition() * tickReverse;
    }

    public void seek(int target){
        double delta = EULMathEx.doubleClamp(0, 1, (target - getCurrentEncoderTicks()) / powerModulator.reactionBand);
        double sign = Math.signum(delta);
        double absDelta = Math.abs(delta);

        storedIntegral += powerModulator.integral(absDelta, 1) * sign; //sums the integrals of the past with the 'present' integral

        //The P term is self-explanatory
        //The I term is using the actual integral and will always seek 0 (meaning that the target has been reached) so we get rid of the trapezoidal rule entirely though we still use a summation
        //The D term is using the actual function's derivative
        double powerOutput = kP * delta + kI * storedIntegral + kD * powerModulator.derivative(absDelta) * sign;
        setPower(powerOutput);

        hasReachedTarget = absDelta <= targetTickTolerance;
    }

    public boolean hasReachedTarget(){
        return hasReachedTarget;
    }

    public boolean isBusy(){
        return Math.abs(motor.getPower()) >= 0.05;
    }
}