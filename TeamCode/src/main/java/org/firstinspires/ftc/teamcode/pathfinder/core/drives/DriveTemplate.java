package org.firstinspires.ftc.teamcode.pathfinder.core.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PathfinderPath;

public abstract class DriveTemplate {

    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;

    public DriveTemplate(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public abstract void manualDrive();
    public abstract void followPath();
    public abstract void buildPath(PathfinderPath path);
}
