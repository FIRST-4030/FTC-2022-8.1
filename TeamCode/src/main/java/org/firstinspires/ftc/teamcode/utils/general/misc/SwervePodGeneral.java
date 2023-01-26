package org.firstinspires.ftc.teamcode.utils.general.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomDrive;

import java.util.HashMap;
import java.util.Objects;

public class SwervePodGeneral extends CustomDrive {

    protected Matrix2d wheelPowerMatrix;
    protected Vector2d out;
    protected double turnCoefficient;
    protected double spinCoefficient;
    protected double coefficientSum;

    public SwervePodGeneral(HardwareMap hardwareMap, double turnCoefficient, double spinCoefficient){
        this.hardwareMap = hardwareMap;
        initImu();

        this.motorMap = new HashMap<>();
        this.turnCoefficient = turnCoefficient;
        this.spinCoefficient = spinCoefficient;

        this.coefficientSum = Math.abs(turnCoefficient) + Math.abs(spinCoefficient);

        initMatrix();
        initVirtualRobot();
    }

    @Override
    protected void initImu() {
        //set up IMU parameters for basic angle tracking
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;

        //pass those parameters to 'imu' when the hardware map fetches the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    public void mapMotors(String spin, boolean reverseS, String turn, boolean reverseT, boolean driveToPosition){
        motorMap.clear();
        motorMap.put("S", hardwareMap.get(DcMotor.class, spin));
        motorMap.put("T", hardwareMap.get(DcMotor.class, turn));

        Objects.requireNonNull(motorMap.get("S")).setDirection(reverseS ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("T")).setDirection(reverseT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        motorMap.get("S").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("S").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorMap.get("T").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("T").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!driveToPosition) {

            motorMap.get("S").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorMap.get("T").setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }else{

            motorMap.get("S").setTargetPosition(0);
            motorMap.get("S").setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorMap.get("T").setTargetPosition(0);
            motorMap.get("T").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
    private void initMatrix(){
        wheelPowerMatrix = (new Matrix2d(new double[][]{
                {0.5,  0.5},
                {-0.5, 0.5}
        }));
    }
    public void calibrateTick(){

    }

    public void update(Vector2d control, boolean fieldCentric, double dt){
        virtualRobot.updateHeading();
        virtualRobot.updateTime(dt);

        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix2d rot = fieldCentric ? Matrix2d.makeRotation(-angles.firstAngle) : new Matrix2d();
        Vector2d rotated = rot.times(control.unaryMinus());
        Vector2d internalControl = new Vector2d(rotated.x, rotated.y);
        out = wheelPowerMatrix.times(internalControl);

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("T")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("S")).setPower(out.y);
    }

}
