package org.firstinspires.ftc.teamcode.pathfinder.core.drives.sample;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathfinder.core.drives.DriveTemplate;
import org.firstinspires.ftc.teamcode.pathfinder.core.motors.MotorizedWheel;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderPath;

public class PathfinderMecanumDrive extends DriveTemplate {

    public enum WHEEL{
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    protected MotorizedWheel fl, fr, bl, br;

    public PathfinderMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
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

    public PathfinderMecanumDrive configureWheel(WHEEL wheel, boolean reverseDir, boolean reverseTick, int targetTickTolerance, double powerLimit, int reactionBand, double functionalBias){
        switch (wheel){
            case FRONT_LEFT:
                fl.setOptions(reverseDir, reverseTick, targetTickTolerance, powerLimit, reactionBand, functionalBias);
                break;
            case FRONT_RIGHT:
                fr.setOptions(reverseDir, reverseTick, targetTickTolerance, powerLimit, reactionBand, functionalBias);
                break;
            case BACK_LEFT:
                bl.setOptions(reverseDir, reverseTick, targetTickTolerance, powerLimit, reactionBand, functionalBias);
                break;
            case BACK_RIGHT:
                br.setOptions(reverseDir, reverseTick, targetTickTolerance, powerLimit, reactionBand, functionalBias);
                break;
        }

        return this;
    }

    @Override
    public void manualDrive() {

    }

    @Override
    public void followPath() {

    }

    @Override
    public void buildPath(PathfinderPath path) {

    }
}
