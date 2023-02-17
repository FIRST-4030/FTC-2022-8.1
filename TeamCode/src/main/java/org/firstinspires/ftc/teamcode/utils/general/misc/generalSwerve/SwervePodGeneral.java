package org.firstinspires.ftc.teamcode.utils.general.misc.generalSwerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.utils.fileRW.main.FileRW;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.csv.CSVLexer;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv.CSVParser;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.csv.CSVWriter;
import org.firstinspires.ftc.teamcode.utils.sensors.pot.BasicPotentiometer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

public class SwervePodGeneral extends CustomDrive {

    public HashMap<String, BasicPotentiometer> potentiometerMap;
    protected Matrix2d wheelPowerMatrix;
    protected Vector2d out;
    protected double turnCoefficient;
    protected double spinCoefficient;
    protected double coefficientSum;
    public ArrayList<Double> potentiometerArray1;
    public ArrayList<Double> potentiometerArray2;
    public ArrayList<Integer> motorTicks;
    protected FileRW dataFile;

    public SwervePodGeneral(HardwareMap hardwareMap, double turnCoefficient, double spinCoefficient){
        this.hardwareMap = hardwareMap;
        initImu();

        this.potentiometerMap = new HashMap<>();
        this.motorMap = new HashMap<>();
        this.turnCoefficient = turnCoefficient;
        this.spinCoefficient = spinCoefficient;

        this.coefficientSum = Math.abs(turnCoefficient) + Math.abs(spinCoefficient);

        this.dataFile = new FileRW(new CSVLexer(), new CSVParser(), new CSVWriter());
        dataFile.init();

        this.potentiometerArray1 = new ArrayList<>();
        this.potentiometerArray2 = new ArrayList<>();
        this.motorTicks = new ArrayList<>();

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

    public void mapPotentiometers(String p1, String p2, Telemetry telemetry){
        potentiometerMap.clear();
        potentiometerMap.put("P1", new BasicPotentiometer(hardwareMap, telemetry, p1, new double[] {0,1}, new double[] {0,1}));
        potentiometerMap.put("P2", new BasicPotentiometer(hardwareMap, telemetry, p2, new double[] {0,1}, new double[] {0,1}));
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
        Objects.requireNonNull(motorMap.get("T")).setPower(-0.5);
        Objects.requireNonNull(motorMap.get("S")).setPower(0.5);
        potentiometerArray1.add(potentiometerMap.get("P1").getMV());
        potentiometerArray2.add(potentiometerMap.get("P2").getMV());
        motorTicks.add(motorMap.get("S").getCurrentPosition());
    }

    public void writeToFile(String name){
        dataFile.init();
        for(int i = 0; i<motorTicks.size(); i++){
            dataFile.writeToRow(i,  dataFile.translateToRow(Arrays.asList(motorTicks.get(i), potentiometerArray1.get(i), potentiometerArray2.get(i))));
        }
        dataFile.finalizeWriteTo(name);
    }

    public void update(Vector2d control, boolean fieldCentric, double dt){
        //virtualRobot.updateHeading();
        //virtualRobot.updateTime(dt);

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
