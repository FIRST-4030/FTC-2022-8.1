package org.firstinspires.ftc.teamcode.pathfinder.core.drives.sample;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.pathfinder.core.drives.DriveTemplate;
import org.firstinspires.ftc.teamcode.pathfinder.core.motors.MotorizedWheel;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.DriveSpec;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderJoystickControlModule;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderPath;
//TODO: Add documentation
public class PathfinderMecanumDrive extends DriveTemplate {

    public enum WHEEL{
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    protected MotorizedWheel fl, fr, bl, br;
    protected PathfinderPath activePath = null;
    protected PathfinderJoystickControlModule joystickControlModule;

    protected Matrix4d powerPartitionMatrix, identityPartitionMatrix;
    protected double biasCoefficientSum = 1;
    protected PathfinderMecanumMotionMaker motionMaker;

    public PathfinderMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, DriveSpec spec){
        super(hardwareMap, telemetry, spec);
        this.motionMaker = new PathfinderMecanumMotionMaker();
        this.motionMaker.bindDrive(spec);
    }

    public void initPowerMatrices(double advancementBias, double lateralBias, double turnBias){
        this.identityPartitionMatrix = (new Matrix4d(new double[][]{
                {1,  1,  1,  0},
                {1, -1, -1,  0},
                {1, -1,  1,  0},
                {1,  1, -1,  0}
        })).times(1/3d);

        biasCoefficientSum = Math.abs(advancementBias) + Math.abs(lateralBias) + Math.abs(turnBias);

        this.powerPartitionMatrix = new Matrix4d(new double[][]{
                {lateralBias,  advancementBias,  turnBias,  0},
                {lateralBias, -advancementBias, -turnBias,  0},
                {lateralBias, -advancementBias,  turnBias,  0},
                {lateralBias,  advancementBias, -turnBias,  0}
        }).times(1 / biasCoefficientSum);
    }

    public PathfinderMecanumDrive tweakMotorPID(WHEEL wheel, double p, double i, double d){
        switch (wheel){
            case FRONT_LEFT:
                fl.setPIDGain(p, i, d);
                break;
            case FRONT_RIGHT:
                fr.setPIDGain(p, i, d);
                break;
            case BACK_LEFT:
                bl.setPIDGain(p, i, d);
                break;
            case BACK_RIGHT:
                br.setPIDGain(p, i, d);
                break;
        }

        return this;
    }

    public PathfinderMecanumDrive mapMotor(WHEEL wheel, String name){
        switch (wheel){
            case FRONT_LEFT:
                fl = new MotorizedWheel(hardwareMap, name);
                break;
            case FRONT_RIGHT:
                fr = new MotorizedWheel(hardwareMap, name);
                break;
            case BACK_LEFT:
                bl = new MotorizedWheel(hardwareMap, name);
                break;
            case BACK_RIGHT:
                br = new MotorizedWheel(hardwareMap, name);
                break;
        }

        return this;
    }

    public PathfinderMecanumDrive configureWheel(WHEEL wheel, boolean reverseDir, boolean reverseTick, int targetTickTolerance, int reactionBand, double functionalBias){
        switch (wheel){
            case FRONT_LEFT:
                fl.setOptions(reverseDir, reverseTick, targetTickTolerance, reactionBand, functionalBias);
                break;
            case FRONT_RIGHT:
                fr.setOptions(reverseDir, reverseTick, targetTickTolerance, reactionBand, functionalBias);
                break;
            case BACK_LEFT:
                bl.setOptions(reverseDir, reverseTick, targetTickTolerance, reactionBand, functionalBias);
                break;
            case BACK_RIGHT:
                br.setOptions(reverseDir, reverseTick, targetTickTolerance, reactionBand, functionalBias);
                break;
        }

        return this;
    }

    @Override
    public void manualDrive() {
        Matrix4d rotMatrix = joystickControlModule.fieldCentric ? Matrix4d.makeAffineRotation(EULMathEx.Axis.AXIS_Z, -joystickControlModule.currentAngle) : new Matrix4d();
        Vector4d powerVectorOutput = powerPartitionMatrix.times(rotMatrix.times(joystickControlModule.getAsVector())).div(Math.max(biasCoefficientSum, 1));

        fl.setPower(powerVectorOutput.x);
        fr.setPower(powerVectorOutput.z);
        bl.setPower(powerVectorOutput.y);
        br.setPower(powerVectorOutput.w);
    }

    @Override
    public void followPath() {

    }

    @Override
    public void buildPath(PathfinderPath path) {
        this.activePath = path;
    }
}
