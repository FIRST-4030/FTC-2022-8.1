package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathfinder.core.motors.MotorizedWheel;

public abstract class DriveCapabilities {

    /**
     * Declares what the hardware is located programmatically (Ideally, it would be the OpMode's hardwareMap)
     */
    public HardwareMap hardwareMap;

    /**
     * Declares the bounded telemetry that the classes should report to (Ideally, it would be the OpMode's telemetry)
     */
    public Telemetry telemetry;

    /**
     * Declares that the robot can move forward and backwards
     */
    public boolean movtForward = true;

    /**
     * Declares that the robot can move laterally (ex: Mecanum drive)
     */
    public boolean movtLateral = false;

    /**
     * Parameter is ticks per meter in the forward/backward
     */
    public double ticksPerMeterFWD = 0;

    /**
     * Parameter is ticks per meter in the lateral direction ('cause we aren't in a frictionless vacuum)
     */
    public double ticksPerMeterLAT = 0;

    /**
     * Parameter is ticks per FULL turn (360 deg; 2π Radians; 1 Turn)
     */
    public double ticksPerTurn = 0;

    public abstract void build();
    public abstract MotorizedWheel[] makeWheels();
}
