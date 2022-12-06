package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement.AnglePID;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;
import org.firstinspires.ftc.teamcode.utils.sensors.distance.RevDistance;

@Config
@TeleOp(name = "PowerPlayMecanum", group = "Test")
public class DriveTest extends LoopUtil {

    public static CustomMecanumDrive drive  = null;
    public static Vector3d joystick = new Vector3d();
    public static Vector2d right_stick = new Vector2d();
    public static AnglePID Apid = null;
    public static double[] angles = null;
    public static int angleIndex = 0;
    public static boolean lastStateLB = false, lastStateRB = false,currentStateLB = false, currentStateRB = false;
    public static double pGain = 0, iGain = 0, dGain = 0;
    public static RevDistance D1, D2;


    //Algorithm-based correction (not PID)
    public static AlgorithmicCorrection correction;

    RevColorRange RCR2;
    ColorView CV2;

    @Override
    public void opInit() {

        Globals.opmode(this);
        Globals.input(this);

        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true, false);

        joystick = new Vector3d();
        right_stick = new Vector2d(0, 1);

        pGain = 1/(Math.PI * 3);
        iGain = 0;
        dGain = 88;

        Apid = new AnglePID(1/Math.PI, 0.000001, 1/4000);
        Apid = new AnglePID(pGain, iGain, dGain);
        angles = new double[]{ 0, Math.PI/2,  Math.PI, -Math.PI/2};

        angleIndex = 0;

        lastStateRB = false;
        lastStateLB = false;
        currentStateLB = false;
        currentStateRB = false;

        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());

        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        D1 = new RevDistance(hardwareMap, telemetry, "range1");
        D2 = new RevDistance(hardwareMap, telemetry, "range2");
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        lastStateRB = currentStateRB;
        lastStateLB = currentStateLB;

        currentStateLB = gamepad1.left_bumper;
        if(currentStateLB && !lastStateLB){
            angleIndex++;
        }

        currentStateRB = gamepad1.right_bumper;
        if(currentStateRB && !lastStateRB){
            angleIndex--;
        }

        if(angleIndex <= -1){
            angleIndex = 3;
        } else if (angleIndex >= 4){
            angleIndex = 0;
        }

        /*
        switch(angleIndex){
            case 0:
                Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                break;
            case 1:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < -1*Math.PI/2 && drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle > -1*Math.PI){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
            case 2:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < 0){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
            case 3:
                if(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle > Math.PI/2 && drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle < Math.PI){
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - 2*Math.PI);
                }else{
                    Apid.update(deltaTime, angles[angleIndex], drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
                }
                break;
        }
         */




        joystick.x = gamepad1.left_stick_x * -1;
        joystick.y = -gamepad1.left_stick_y * -1;

        telemetry.addData("Joystick X: ", joystick.x);
        telemetry.addData("Joystick Y: ", joystick.y);



        right_stick.x = -gamepad1.right_stick_x;
        right_stick.y = gamepad1.right_stick_y;


        // atan2 is x-axis 0
        //right_stick.x = gamepad1.right_stick_x;
        //right_stick.y = gamepad1.right_stick_y;

        correction.update( drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, right_stick, false);
        //correction.update( drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, angles[angleIndex], false);

        //joystick.z = gamepad1.right_stick_x; //manual steering

       // joystick.z = -1*Apid.correctionPower; //correction steering (PID)

        joystick.z = correction.getOutput(); //an algorithm directly controls the rotation instead of adding to it



        CV2.update(RCR2.color(), RCR2.distance());

        drive.update(joystick, true, deltaTime);
        telemetry.addData("Angle: ", drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Acceleration: ", drive.getImu().getLinearAcceleration());
        AngularVelocity avel = drive.getImu().getAngularVelocity();
        telemetry.addData("Turn Velocity: ", new Vector3d(avel.xRotationRate, avel.yRotationRate, avel.zRotationRate));
        telemetry.addData("Angle Index: ", angleIndex);
        telemetry.addData("P: ", Apid.p);
        telemetry.addData("I: ", Apid.i);
        telemetry.addData("D: ", Apid.d);
        telemetry.addData("Correction: ", Apid.correctionPower);
        telemetry.addData("ColorBetter: ", CV2.getColorBetter(80));
        telemetry.addData("Color: ", CV2.getColor());
        telemetry.addData("Color: ", CV2.convertRGBToHSV(CV2.colorInput)[0]);
        telemetry.addData("Red: ", RCR2.color().red);
        telemetry.addData("Blue: ", RCR2.color().blue);
        telemetry.addData("Green: ", RCR2.color().green);
        telemetry.addData("Distance: ", RCR2.distance());
        telemetry.addData("Distance Sensor 1: ", D1.distance(DistanceUnit.CM));
        telemetry.addData("Distance Sensor 2: ", D2.distance(DistanceUnit.CM));
        telemetry.addData("Angle to Wall: ", Math.atan((D1.distance(DistanceUnit.CM) - D2.distance(DistanceUnit.CM))/18) * EULConstants.RAD2DEG);
        telemetry.addData("Angle to Wall (Atan2): ", Math.atan2(D2.distance(DistanceUnit.CM) - D1.distance(DistanceUnit.CM), 18) * EULConstants.RAD2DEG);

        correction.log(telemetry);
    }

    @Override
    public void opStop() {

    }
}
