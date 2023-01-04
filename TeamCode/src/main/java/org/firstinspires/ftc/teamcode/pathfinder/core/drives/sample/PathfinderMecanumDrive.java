package org.firstinspires.ftc.teamcode.pathfinder.core.drives.sample;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.pathfinder.core.drives.DriveTemplate;
import org.firstinspires.ftc.teamcode.pathfinder.core.motors.MotorizedWheel;
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

    public PathfinderMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, double advancementTickPerMeasurementUnit, double lateralTickPerMeasurementUnit){
        super(hardwareMap, telemetry, advancementTickPerMeasurementUnit, lateralTickPerMeasurementUnit);
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

    }

    @Override
    public void followPath() {

    }

    @Override
    public void buildPath(PathfinderPath path) {
        this.activePath = path;
    }

    protected double calcRate(Vector2d dir){
        return (advancementTickPerMeasurementUnit * lateralTickPerMeasurementUnit) /
                (Math.sqrt(dir.y * lateralTickPerMeasurementUnit * dir.y * lateralTickPerMeasurementUnit +
                        dir.x * advancementTickPerMeasurementUnit * dir.x * advancementTickPerMeasurementUnit));
    }
}
