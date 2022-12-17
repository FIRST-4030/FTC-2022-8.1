package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AlgorithmicCorrection;

public class SlideController {

    public enum LEVEL{
        REST, LOW, MIDDLE, HIGH
    }

    private DcMotor right;

    public int target;

    public AlgorithmicCorrection.BiasedInterpolation BiasMath;

    public int rightEncoderPosition = 0;
    public double powerOutput = 0;

    private boolean inUse = false;
    public LEVEL currentLevel = LEVEL.REST;

    public SlideController(HardwareMap hardwareMap, String rightMotorName, boolean invertRight){
        right = hardwareMap.dcMotor.get(rightMotorName);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BiasMath = new AlgorithmicCorrection.BiasedInterpolation(0.7);
    }

    public SlideController(HardwareMap map){}

    public void update(double deltaTime, LEVEL level, double slidePower){

        rightEncoderPosition = right.getCurrentPosition();
        currentLevel = level;

        switch (level){
            case REST:
                target = 0;
                break;
            case LOW:
                target = (540 / 3 - 50);
                break;
            case MIDDLE:
                target = (540 / 3 + 235);
                break;
            case HIGH:
                target = (540 / 3 + 290);
                break;
        }

        double t = EULMathEx.doubleClamp(0, 1, (1-(Math.abs(target - rightEncoderPosition))/300d));
        double p = Math.signum(target - rightEncoderPosition)*BiasMath.process(t);
        double d = BiasMath.derivative(t);
        powerOutput = 0.6 * p + 0.4 * d;
        if((level == LEVEL.REST && rightEncoderPosition < 5) || Math.abs(powerOutput) < 0.05){
            powerOutput = 0;
        }
        //left.setPower(0);

        inUse = !(level == LEVEL.REST);
    }



/*
    public void update(double deltaTime, LEVEL level, double slidePower){

        leftEncoderPosition = left.getCurrentPosition();
        rightEncoderPosition = right.getCurrentPosition();

        switch (level){
            case REST:
                left.setTargetPosition(0);
                right.setTargetPosition(0);
                break;
            case LOW:
                left.setTargetPosition(540 / 3 - 50);
                right.setTargetPosition(540 / 3 - 50);
                break;
            case MIDDLE:
                left.setTargetPosition(540 / 3 + 100);
                right.setTargetPosition(540 / 3 + 100);
                break;
            case HIGH:
                left.setTargetPosition(540 / 3 + 250);
                right.setTargetPosition(540 / 3 + 250);
                break;
        }

        if (Math.abs(rightLastEncoderPosition - right.getTargetPosition()) >= tickTolerance) {
            left.setPower(Math.abs(rightLastEncoderPosition - right.getTargetPosition())/50f);
            right.setPower(Math.abs(rightLastEncoderPosition - right.getTargetPosition())/50f);
        } else {
            left.setPower(0.5);
            left.setTargetPosition(leftEncoderPosition);
            right.setPower(0.5);
            right.setTargetPosition(rightEncoderPosition);
        }

        inUse = !(level == LEVEL.REST);

        leftLastEncoderPosition = leftEncoderPosition;
        rightLastEncoderPosition = rightEncoderPosition;

    }

 */


    public void setPower(double power){
        right.setPower(power);
    }

    public void logMotorPos(Telemetry telemetry){
        telemetry.addData("LSRM Encoder Position: ", right.getCurrentPosition());
    }

    public boolean isInUse(){
        return inUse;
    }

    public DcMotor getRight(){
        return right;
    }
}
