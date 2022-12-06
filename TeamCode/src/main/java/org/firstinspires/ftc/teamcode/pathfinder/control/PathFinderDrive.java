package org.firstinspires.ftc.teamcode.pathfinder.control;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PFPath;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PFPose2d;

public abstract class PathFinderDrive{

    public static class Parameters{
        public BNO055IMU.Parameters imuParams;
    }

    protected BNO055IMU imu;
    protected Parameters parameters;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public PathFinderDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    protected abstract void setupInternals();

    /**
     * This method sets the IMU up for use
     */
    protected void setupImu(){
        BNO055IMU.Parameters imuParams = parameters.imuParams;
        this.imu = hardwareMap.get(AdafruitBNO055IMU.class, "imu");
        this.imu.initialize(imuParams);
    }

    public abstract void followPath();

    public abstract void joystickControl(Object... args);

    public double getCurrentTurnAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public Orientation getOrientation(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public abstract void buildPath(PFPath path);

    public PFPath makeNewPath(PFPose2d initialPose){
        return new PFPath(this, initialPose);
    }

    public abstract PathFinderDrive setFollowingPath(PFPath nPath);
}
