package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.accelintegration.AccelIntegratorSemiImplicitEuler;

import java.util.HashMap;
import java.util.Objects;
import java.util.Stack;

public class MecanumMovementFactory {

    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    private HashMap<String, DcMotor> motorMap;
    private Matrix4d mecanumPowerRatioMatrix;
    private double forwardBackMovt, strafeMovt, turnMovt;
    private double coefficientSum;
    private Vector3d modulation = new Vector3d();
    private double acceptableError = 0.05;

    private Position recordedPos = new Position();
    public Vector4d out;
    public AccelIntegratorSemiImplicitEuler sme;



    //sequential execution of commands
    public boolean isDone;
    public double elapsed_time;
    public Stack<Pair<String, Double>> cmdStack;
    //public Stack<Pair<String, Double>> cmdCache;

    public MecanumMovementFactory(HardwareMap hardwareMap, double forwardBackCoefficient, double strafingCoefficient, double turnCoefficient){
        this.hardwareMap = hardwareMap;
        initIMU(hardwareMap);

        motorMap = new HashMap<>();
        forwardBackMovt = forwardBackCoefficient;
        strafeMovt = strafingCoefficient;
        turnMovt = turnCoefficient;

        coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);

        initMatrix();

        initCMD();
    }

    public void mapMotors(String frontLeft, boolean reverseFL, String backLeft, boolean reverseBL, String frontRight, boolean reverseFR, String backRight, boolean reverseBR){
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
        motorMap.get("FL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("FL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("FR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("FR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("FR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("BL").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BL").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("BL").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorMap.get("BR").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMap.get("BR").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMap.get("BR").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Vector3d control, boolean fieldCentric){
        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix3d rot = fieldCentric ? Matrix3d.makeAffineRotation(-angles.firstAngle) : new Matrix3d();
        Vector3d internalControl = rot.times(new Vector3d(control.x * strafeMovt, control.y * forwardBackMovt, control.z * turnMovt));
        Vector4d in = new Vector4d(internalControl.x, internalControl.y, internalControl.z, 0);
        out = mecanumPowerRatioMatrix.times(in).div(Math.max(coefficientSum, 1));

        //set the motor powers as referenced in the hashmap
        Objects.requireNonNull(motorMap.get("FL")).setPower(out.x);
        Objects.requireNonNull(motorMap.get("BL")).setPower(out.y);
        Objects.requireNonNull(motorMap.get("FR")).setPower(out.z);
        Objects.requireNonNull(motorMap.get("BR")).setPower(out.w);
    }

    public void update(){
        update(modulation, true);
    }

    public Position getPos(){
        return imu.getPosition();
    }

    public boolean alignX(Vector2d input){

        Position currentPos = imu.getPosition();
        Velocity currentVelocity = imu.getVelocity();
        boolean outputBool = (input.x - currentPos.x) <= acceptableError;

        calcPos(input);

        Vector3d output = new Vector3d();
        output.x = modulation.x;
        output.y = 0;
        output.z = 0;
        /*
        if(!outputBool) {
            update(output, true);
        }

         */
        return outputBool;
    }

    public boolean alignY(Vector2d input){
        Position currentPos = imu.getPosition();
        Velocity currentVelocity = imu.getVelocity();
        boolean outputBool = (input.x - currentPos.x) <= acceptableError;

        calcPos(input);

        Vector3d output = new Vector3d();
        output.x = 0;
        output.y = modulation.y;
        output.z = 0;

        /*
        update(output, true);

         */
        return outputBool;
    }

    private void calcPos(Vector2d target){
        sme.update(imu.getLinearAcceleration());
        Vector2d delta = new Vector2d();
        Position integratedPos = sme.getPosition();

        integratedPos.x -= recordedPos.x;
        integratedPos.y -= recordedPos.y;
        integratedPos.z -= recordedPos.z;

        delta.x = target.x - integratedPos.x;
        delta.y = target.y - integratedPos.y;
        double distance =  delta.length();
        boolean far = distance > acceptableError * 2;

        modulation.x = EULMathEx.doubleClamp(-1, 1, far ? delta.x / distance : delta.x);
        modulation.y = EULMathEx.doubleClamp(-1, 1, far ? delta.y / distance : delta.y);

    }

    //cmd queueing methods



    public void execute(double dt){
        if(isDone && cmdStack.size() != 0){ cmdStack.pop(); elapsed_time = 0; modulation.x = 0; modulation.y = 0; modulation.z = 0; }
        Pair<String, Double> cmd = cmdStack.peek();
        elapsed_time += dt;

        if(elapsed_time >= cmd.snd && cmd.snd != -1){ isDone = true; }
        else {
            switch (cmd.fst) {
                case "forward":
                    isDone = false;
                    modulation.y = -1;
                    break;
                case "back":
                    isDone = false;
                    modulation.y = 1;
                    break;
                case "left":
                    isDone = false;
                    modulation.x = 1;
                    break;
                case "right":
                    isDone = false;
                    modulation.x = -1;
                    break;
                case "turnLeft":
                    isDone = false;
                    modulation.z = 1;
                    break;
                case "turnRight":
                    isDone = false;
                    modulation.z = -1;
                    break;
                case "idle":
                    isDone = false;
                    modulation.x = 0;
                    modulation.y = 0;
                    modulation.z = 0;
                    break;
            }
        }

        update(modulation, true);
    }

    private void initIMU(HardwareMap hardwareMap){
        //set up IMU parameters for basic angle tracking
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imuParams.accelerationIntegrationAlgorithm = new AccelIntegrationVerlet();
        imuParams.loggingEnabled = false;
        long time = System.nanoTime();
        sme = new AccelIntegratorSemiImplicitEuler();
        sme.initialize(imuParams, new Position(DistanceUnit.METER, 0, 0, 0, time), new Velocity(DistanceUnit.METER, 0, 0, 0, time));

        //pass those parameters to 'imu' when the hardware map fetches the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);


        //imu.startAccelerationIntegration(new Position(DistanceUnit.METER, 0, 0, 0, time), new Velocity(DistanceUnit.METER, 0, 0, 0, time), 5);
        recordedPos = imu.getPosition();
    }

    private void initMatrix(){
        //4th column discards W component of the Vector4f multiplied with the matrix
        mecanumPowerRatioMatrix = new Matrix4d(new double[][]{
                {1,  1,  1, 0},
                {1, -1, -1, 0},
                {1, -1,  1, 0},
                {1,  1, -1, 0}
        });
    }

    private void initCMD(){
        isDone = false;
        elapsed_time = 0;
        //cmdStack = new Stack<>();
        //cmdCache = new Stack<>();
    }

    private void initCoefficients(){
        coefficientSum = Math.abs(forwardBackMovt) + Math.abs(strafeMovt) + Math.abs(turnMovt);
    }

    public void dispose(){
        imu.stopAccelerationIntegration();
    }
}
