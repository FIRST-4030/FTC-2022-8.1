package org.firstinspires.ftc.teamcode.pathfinder.core.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderPath;
//TODO: Add documentation
public abstract class DriveTemplate {

    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected double advancementTickPerMeasurementUnit, lateralTickPerMeasurementUnit;

    public DriveTemplate(HardwareMap hardwareMap, Telemetry telemetry, double advancementTickPerMeasurementUnit, double lateralTickPerMeasurementUnit){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.advancementTickPerMeasurementUnit = advancementTickPerMeasurementUnit;
        this.lateralTickPerMeasurementUnit = lateralTickPerMeasurementUnit;
    }

    public abstract void manualDrive();
    public abstract void followPath();
    public abstract void buildPath(PathfinderPath path);
}
