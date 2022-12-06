package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.general.misc.VirtualRobot;

import java.util.HashMap;

public abstract class CustomDrive {
    public String name;

    protected HardwareMap hardwareMap;
    protected BNO055IMU imu;
    protected HashMap<String, DcMotor> motorMap;
    protected VirtualRobot virtualRobot;

    protected abstract void initImu();

    protected void initVirtualRobot(){
        this.virtualRobot = new VirtualRobot(motorMap, imu);
    }

    public BNO055IMU getImu(){return imu;}
    public HashMap<String, DcMotor> getMotorMap(){return motorMap;}
    public VirtualRobot getVirtualRobot(){return virtualRobot;}
}
