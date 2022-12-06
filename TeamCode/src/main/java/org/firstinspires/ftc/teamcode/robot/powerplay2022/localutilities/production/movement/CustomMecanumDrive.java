package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement.MecanumDriveState;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement.MecanumDriveTrajectory;

import java.util.HashMap;
import java.util.Objects;

public class CustomMecanumDrive extends CustomDrive{

    /*
    protected HardwareMap hardwareMap;
    protected BNO055IMU imu;
     */

    //protected HashMap<String, DcMotor> motorMap;

    //mecanum specific stuff
    protected Matrix4d mecanumPowerRatioMatrix;
    protected double forwardBackMovt, strafeMovt, turnMovt;
    protected double coefficientSum;
    protected MecanumDriveTrajectory followTrajectory;
    protected boolean fieldCentricMode = true;
    protected int[] savedTicks = new int[4];

    protected Vector4d out;
    //public double deltaTime;

    private double outputMultiplier = 1;

    public CustomMecanumDrive(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        initImu();

        this.motorMap = new HashMap<>();
        this.forwardBackMovt = forwardBackCoefficient;
        this.strafeMovt = strafingCoefficient;
        this.turnMovt = turnCoefficient;

        this.coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);

        initMatrix();
        //this.deltaTime = 1;

        initVirtualRobot();
    }

    public void mapMotors(String frontLeft, boolean reverseFL, String backLeft, boolean reverseBL, String frontRight, boolean reverseFR, String backRight, boolean reverseBR, boolean driveToPosition){
        motorMap.clear();
        motorMap.put("FL", hardwareMap.get(DcMotor.class, frontLeft));
        motorMap.put("FR", hardwareMap.get(DcMotor.class, frontRight));
        motorMap.put("BL", hardwareMap.get(DcMotor.class, backLeft));
        motorMap.put("BR", hardwareMap.get(DcMotor.class, backRight));

        Objects.requireNonNull(motorMap.get("FL")).setDirection(reverseFL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("FR")).setDirection(reverseFR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("BL")).setDirection(reverseBL ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        Objects.requireNonNull(motorMap.get("BR")).setDirection(reverseBR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        motorMap.get("FL").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("FL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorMap.get("FR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("FR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorMap.get("BL").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorMap.get("BR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!driveToPosition) {

            motorMap.get("FL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorMap.get("FR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorMap.get("BL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorMap.get("BR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }else{

            motorMap.get("FL").setTargetPosition(0);
            motorMap.get("FL").setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorMap.get("FR").setTargetPosition(0);
            motorMap.get("FR").setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorMap.get("BL").setTargetPosition(0);
            motorMap.get("BL").setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorMap.get("BR").setTargetPosition(0);
            motorMap.get("BR").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }

    public void update(Vector3d control, boolean fieldCentric, double dt){
        virtualRobot.updateHeading();
        virtualRobot.updateTime(dt);

        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix3d rot = fieldCentric ? Matrix3d.makeAffineRotation(-angles.firstAngle) : new Matrix3d();
        Vector3d rotated = rot.times(control.unaryMinus());
        Vector4d internalControl = new Vector4d(rotated.x, rotated.y, rotated.z, 1);
        out = mecanumPowerRatioMatrix.times(internalControl).div(Math.max(coefficientSum, 1));

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);
    }
    public void moveToPos(Vector3d control){
        savedTicks[0] = Objects.requireNonNull(motorMap.get("FL")).getCurrentPosition() + (int)(control.x*0 + control.y*-1739 + control.z*1013/90); //Change in FL ticks
        savedTicks[1] = Objects.requireNonNull(motorMap.get("FR")).getCurrentPosition() + (int)(control.x*0 + control.y*1739 + control.z*1013/90); //Change in FR ticks
        savedTicks[2] = Objects.requireNonNull(motorMap.get("BL")).getCurrentPosition() + (int)(control.x*0 + control.y*1739 + control.z*-1013/90); //Change in BL ticks
        savedTicks[3] = Objects.requireNonNull(motorMap.get("BR")).getCurrentPosition() + (int)(control.x*0 + control.y*-1739 + control.z*-1013/90); //Change in BR ticks
    }
    public void posUpdate(double x){
        Objects.requireNonNull(motorMap.get("FL")).setTargetPosition(savedTicks[0]);
        Objects.requireNonNull(motorMap.get("FR")).setTargetPosition(savedTicks[1]);
        Objects.requireNonNull(motorMap.get("BL")).setTargetPosition(savedTicks[2]);
        Objects.requireNonNull(motorMap.get("BR")).setTargetPosition(savedTicks[3]);

        Objects.requireNonNull(motorMap.get("FL")).setPower(x);
        Objects.requireNonNull(motorMap.get("FR")).setPower(x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(x);
        Objects.requireNonNull(motorMap.get("BR")).setPower(x);
    }

    public void followTrajectory(double dt){
        virtualRobot.updateHeading();
        virtualRobot.updateTime(dt);
        MecanumDriveState state = followTrajectory.getCurrentStateStack().peek();
        if(state.isDone()){
            if (!followTrajectory.isCurrentStateStackEmpty()) {
                followTrajectory.pop();
                virtualRobot.virtualJoystick.x = 0;
                virtualRobot.virtualJoystick.y = 0;
                virtualRobot.virtualJoystick.z = 0;
            }
        } else {
            state.update(virtualRobot);
        }

        update(virtualRobot.virtualJoystick, fieldCentricMode, dt);
    }

    @Override
    protected void initImu(){
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

    private void initMatrix(){
        //4th column discards W component of the Vector4f multiplied with the matrix
        mecanumPowerRatioMatrix = (new Matrix4d(new double[][]{
                {strafeMovt,  forwardBackMovt,  turnMovt, 0},
                {strafeMovt, -forwardBackMovt, -turnMovt, 0},
                {strafeMovt, -forwardBackMovt,  turnMovt, 0},
                {strafeMovt,  forwardBackMovt, -turnMovt, 0}
        }).times(1/coefficientSum).times(-outputMultiplier));
    }

    private void initCoefficients(){
        coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);
    }

    public double getOutputMultiplier(){
        return outputMultiplier;
    }

    public void setOutputMultiplier(double nPower){
        this.outputMultiplier = EULMathEx.doubleClamp(-1, 1, nPower);
        initMatrix();
    }

    public BNO055IMU getImu(){
        return imu;
    }

    public void logMotorPos(Telemetry telemetry){
        telemetry.addData("FL Encoder Position: ", Objects.requireNonNull(motorMap.get("FL")).getCurrentPosition());
        telemetry.addData("FR Encoder Position: ", Objects.requireNonNull(motorMap.get("FR")).getCurrentPosition());
        telemetry.addData("BL Encoder Position: ", Objects.requireNonNull(motorMap.get("BL")).getCurrentPosition());
        telemetry.addData("BR Encoder Position: ", Objects.requireNonNull(motorMap.get("BR")).getCurrentPosition());
        telemetry.addData("FL Encoder Target: ", Objects.requireNonNull(motorMap.get("FL")).getTargetPosition());
        telemetry.addData("Saved Ticks: ", savedTicks[0]);
    }


    public DcMotor getMotor(int number){
        DcMotor motor;
        switch (number){
            case 0:
                motor = motorMap.get("FL");
                break;
            case 1:
                motor = motorMap.get("BL");
                break;
            case 2:
                motor = motorMap.get("FR");
                break;
            case 3:
                motor = motorMap.get("BR");
                break;
            default:
                motor = null;
        }

        return motor;
    }
}
