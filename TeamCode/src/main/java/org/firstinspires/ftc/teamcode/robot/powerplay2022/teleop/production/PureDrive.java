package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

/**
 * A very simple OpMode for driving around a Mecanum
 */
@Config
@TeleOp(name = "Mecanum Drive Tester", group = "Tester")
public class PureDrive extends LoopUtil {

    public static InputHandler gamepadHandler;
    public static Vector3d joystick;
    public static CustomMecanumDrive drive;
    public static boolean fieldCentricMode;
    public static double outputSpeed;
    public static double decimalPlace;
    public static double decimalPlaceLowerBound;

    private static String className = PureDrive.class.getSimpleName();

    @Override
    public void opInit() {
        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        joystick = new Vector3d();
        fieldCentricMode = true;
        outputSpeed = 1;
        decimalPlace = 1;

        drive = new CustomMecanumDrive(hardwareMap, 1, 1, 1);
        //drive.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true, false);
        drive.setOutputMultiplier(-1);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        gamepadHandler.loop(); //update gamepads

        joystick.x = gamepadHandler.value("D1:LS_X");
        joystick.y = -gamepadHandler.value("D1:LS_Y");
        joystick.z = -gamepadHandler.value("D1:RS_X");

        handleInput();

        //I don't use drive.setOutputMultiplier because there is no way I waste CPU cycles on rebuilding the matrix

        drive.update(joystick.times(outputSpeed), fieldCentricMode, deltaTime);

        logData();
    }

    public void handleInput(){
        //simple toggle between field centric and relative mode
        //used gamepads.up() to give the ability to buffer the input instead of it reacting right away
        if (gamepadHandler.up("D1:LT")){
            fieldCentricMode = !fieldCentricMode;
        }

        //edit speed on the fly using the DPAD
        if (gamepadHandler.up("D1:DPAD_LEFT")){ //shifts the modified place left by 1
            decimalPlace *= 10;
            decimalPlace = EULMathEx.doubleClamp(decimalPlaceLowerBound, 1, decimalPlace);
        }
        if (gamepadHandler.up("D1:DPAD_RIGHT")){ //shifts the modified place right by 1
            decimalPlace /= 10;
            decimalPlace = EULMathEx.doubleClamp(decimalPlaceLowerBound, 1, decimalPlace);
        }
        if (gamepadHandler.up("D1:DPAD_UP")){ //increase outputSpeed by decimalPlace
            outputSpeed += decimalPlace;
            outputSpeed = EULMathEx.doubleClamp(0, 1, outputSpeed);
        }
        if (gamepadHandler.up("D1:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace
            outputSpeed -= decimalPlace;
            outputSpeed = EULMathEx.doubleClamp(0, 1, outputSpeed);
        }

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }

    public void logData(){
        telemetry.addData(className, "log BEGIN"); //log header for this OpMode
        telemetry.addData("Field Centric Mode: ", fieldCentricMode ? "WORLD_SPACE" : "RELATIVE_SPACE");
        telemetry.addData("Drive Speed: ", outputSpeed);
        telemetry.addData("Place: ", decimalPlace);
        telemetry.addData("", drive.getVirtualRobot().toString());
        telemetry.addData("Joystick Angle: ", Math.atan2(gamepadHandler.value("D1:RS_X"), -gamepadHandler.value("D1:RS_Y")));
        drive.logMotorPos(telemetry);
        telemetry.addData(className, "log END"); //log footer for this OpMode
    }
}
