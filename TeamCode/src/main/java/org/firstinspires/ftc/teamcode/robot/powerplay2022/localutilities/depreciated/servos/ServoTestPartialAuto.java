package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.servos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Disabled
@TeleOp(name = "Arm Servo Test 2", group = "Tester")
public class ServoTestPartialAuto extends LoopUtil {


    public static ServoFTC servoA, servoB, servoC;
    public static ServoConfig configA, configB, configC;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;
    public static double commandedPositionMultiplier;
    public double speed; //cm/s
    public Vector2d target;
    public double Arm1, Arm2, vt;
    public double sub1, sub2, angleA, angleB, angleC;

    public static ServoAngleConversion servoConversionA, servoConversionB;

    @Override
    public void opInit() {
        configA = new ServoConfig("A",false, 0, 0.75);
        configB = new ServoConfig("B",false, 0, 1);
        configC = new ServoConfig("C",false, 0, 1);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);

        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        enableJoystick = false;
        commandedPosition = 0.0;
        commandedPositionMultiplier = 1;

        speed = 1;
        target = new Vector2d(15, 15);
        Arm1 = 15;
        Arm2 = 13;
        vt = Math.sqrt(target.x*target.x + target.y*target.y);
        sub1 = 0;
        sub2 = 0;
        angleA = 0;
        angleB = 0;
        angleC = 0;

        servoConversionA = new ServoAngleConversion(0, 180, ServoAngleConversion.ANGLE_UNIT.DEGREES);
        servoConversionB = new ServoAngleConversion(0, 270, ServoAngleConversion.ANGLE_UNIT.DEGREES);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        gamepadHandler.loop();

        if (gamepadHandler.up("D1:DPAD_UP")){ //increase outputSpeed by decimalPlace
            target.y -= deltaTime * EULConstants.MS2SEC * speed;
        }
        if (gamepadHandler.up("D1:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace
            target.y += deltaTime * EULConstants.MS2SEC * speed;
        }

        if (gamepadHandler.up("D1:DPAD_LEFT")){ //increase outputSpeed by decimalPlace
            target.x -= deltaTime * EULConstants.MS2SEC * speed;
        }
        if (gamepadHandler.up("D1:DPAD_RIGHT")){ //decrease outputSpeed by decimalPlace
            target.x += deltaTime * EULConstants.MS2SEC * speed;
        }

        if(target.length() > 26){ target = target.normalized().times(26); }

        sub1 = Math.abs(Math.atan2(target.y, target.x));
        sub2 = Math.acos((Arm1*Arm1 + vt*vt - Arm2*Arm2)/2*Arm1*vt);
        angleA = sub1 + sub2;

        servoConversionA.angle2Scalar(90);
        servoA.setPosition(servoConversionA.getOutput());
        servoConversionB.angle2Scalar(135 - 90);
        servoB.setPosition(servoConversionB.getOutput());

        telemetry.addData("Commanded Multiplier: ", commandedPositionMultiplier);
        telemetry.addData("Commanded Position: ", commandedPosition);
        telemetry.addData("Servo Turn Position: ", servoA.getPosition());
        telemetry.addData("Servo Turn Position: ", servoB.getPosition());
        telemetry.addData("Servo Turn Position: ", servoC.getPosition());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
