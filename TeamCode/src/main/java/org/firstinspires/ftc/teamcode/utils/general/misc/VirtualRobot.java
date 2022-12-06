package org.firstinspires.ftc.teamcode.utils.general.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;

import java.util.HashMap;

public class VirtualRobot {

    //heading information
    private double headingAngle;
    private Vector2d xAxis;
    private Vector2d yAxis;
    private Matrix3d transform;

    //general information
    private double deltaTime;
    private HashMap<String, DcMotor> motorMap;
    private BNO055IMU imu;

    //joystick vector (X, Y map to the either side's gamepad stick and Z tells rotation)
    public Vector3d virtualJoystick;

    /**
     * This constructor takes in a previously filled hashmap of all motor references used in driving while the imu is pre-initialized with desired settings
     * @param motorMap
     * @param bno055IMU
     */
    public VirtualRobot(HashMap<String, DcMotor> motorMap, BNO055IMU bno055IMU){
        //motor mapping
        this.motorMap = motorMap;

        //imu
        this.imu = bno055IMU;

        //zero out the unknowns
        this.headingAngle = 0;
        this.xAxis = new Vector2d(1, 0);
        this.yAxis = new Vector2d(0, 1);

        this.deltaTime = 0;

        this.virtualJoystick = new Vector3d();
    }

    /**
     * Called whenever you need the imu measurement for heading at the current time step
     */
    public void updateHeading(){
        headingAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        updateMatrix();
    }

    /**
     * This method is intended to be called once a loop to update the time variables stored (lastTime, currentTime, deltaTime)
     */
    public void updateTime(double deltaTime){
        this.deltaTime = deltaTime;
    }

    private void updateMatrix(){
        transform = Matrix3d.makeAffineRotation(headingAngle);
        xAxis = transform.times(new Vector3d(1, 0, 0)).getXY();
        yAxis = transform.times(new Vector3d(0, 1, 0)).getXY();
    }

    public String toString(){
        return "Heading Angle: " + headingAngle + imu.getParameters().angleUnit.toString() + "s\n"
                + "Delta Time: " + deltaTime + "ms";
    }

    public double getHeadingAngle() {
        return headingAngle;
    }

    public Vector2d getxAxis() {
        return xAxis;
    }

    public Vector2d getyAxis() {
        return yAxis;
    }

    public Matrix3d getTransform() {
        return transform;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public HashMap<String, DcMotor> getMotorMap() {
        return motorMap;
    }

    public BNO055IMU getImu() {
        return imu;
    }
}
