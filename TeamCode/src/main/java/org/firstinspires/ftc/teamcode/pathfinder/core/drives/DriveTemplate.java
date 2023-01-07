package org.firstinspires.ftc.teamcode.pathfinder.core.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderJoystickControlModule;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderPath;

//TODO: Add documentation
public abstract class DriveTemplate {

    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected double advancementTickPerMeasurementUnit, lateralTickPerMeasurementUnit, ticksPerTurn;
    protected PathfinderJoystickControlModule joystickControlModule;

    public DriveTemplate(HardwareMap hardwareMap, Telemetry telemetry, double advancementTickPerMeasurementUnit, double lateralTickPerMeasurementUnit, double ticksPerTurn){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.advancementTickPerMeasurementUnit = advancementTickPerMeasurementUnit;
        this.lateralTickPerMeasurementUnit = lateralTickPerMeasurementUnit;
        this.ticksPerTurn = ticksPerTurn;
    }

    public void setJoystickControlModule(PathfinderJoystickControlModule module){
        this.joystickControlModule = module;
    }

    public abstract void manualDrive();
    public abstract void followPath();
    public abstract void buildPath(PathfinderPath path);
}
