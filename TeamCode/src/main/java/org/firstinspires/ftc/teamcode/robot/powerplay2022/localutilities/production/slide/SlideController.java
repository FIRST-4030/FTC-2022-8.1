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

    private DcMotor left, right;

    public int target;

    public AlgorithmicCorrection.BiasedInterpolation BiasMath;

    public int leftEncoderPosition = 0;
    public int rightEncoderPosition = 0;
    public int tickTolerance = 20;

    private int leftLastEncoderPosition = 0;
    private int rightLastEncoderPosition = 0;

    private boolean inUse = false;

    public SlideController(HardwareMap hardwareMap, String leftMotorName, boolean invertLeft, String rightMotorName, boolean invertRight){
        left = hardwareMap.dcMotor.get(leftMotorName);
        right = hardwareMap.dcMotor.get(rightMotorName);


        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(invertLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BiasMath = new AlgorithmicCorrection.BiasedInterpolation(0.7);
    }

    public SlideController(HardwareMap hardwareMap){}

    public void update(double deltaTime, LEVEL level, double slidePower){

        leftEncoderPosition = left.getCurrentPosition();
        rightEncoderPosition = right.getCurrentPosition();

        switch (level){
            case REST:
                target = 0;
                break;
            case LOW:
                target = (540 / 3 - 50);
                break;
            case MIDDLE:
                target = (540 / 3 + 100);
                break;
            case HIGH:
                target = (540 / 3 + 250);
                break;
        }

        left.setPower(BiasMath.process(EULMathEx.doubleClamp(-1, 1, (target - getLeft().getCurrentPosition())/(double) target)));

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
        left.setPower(power);
        right.setPower(power);
    }

    public void logMotorPos(Telemetry telemetry){
        telemetry.addData("LSRM Encoder Position: ", right.getCurrentPosition());
        telemetry.addData("LSLM Encoder Position: ", left.getCurrentPosition());
    }

    public boolean isInUse(){
        return inUse;
    }

    public DcMotor getLeft(){
        return left;
    }

    public DcMotor getRight(){
        return right;
    }
}
