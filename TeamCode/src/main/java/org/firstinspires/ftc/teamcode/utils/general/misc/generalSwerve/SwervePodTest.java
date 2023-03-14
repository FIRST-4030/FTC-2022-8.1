package org.firstinspires.ftc.teamcode.utils.general.misc.generalSwerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Config
@TeleOp(name = "Swerve Pod Test", group = "Tester")
public class SwervePodTest extends LoopUtil {
    public static InputHandler gamepadHandler;
    public static Vector2d joystickLeft;
    public static SwervePodGeneral driveLeft;
    public static Vector2d joystickRight;
    public static SwervePodGeneral driveRight;
    public double startTimeInit;


    @Override
    public void opInit() {
        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        joystickLeft = new Vector2d();
        driveLeft = new SwervePodGeneral(hardwareMap, 1, 1);
        driveLeft.mapMotors("p1m1", true, "p1m2", false, false);
        driveLeft.mapPotentiometers("pot1a", "pot1b", telemetry);
        joystickRight = new Vector2d();
        driveRight = new SwervePodGeneral(hardwareMap, 1, 1);
        driveRight.mapMotors("p2m1", true, "p2m2", false, false);
        startTimeInit = System.currentTimeMillis();
    }

    @Override
    public void opInitLoop() {
        if(System.currentTimeMillis() - startTimeInit < 30000){
            driveLeft.calibrateTick();
        }else{
            driveLeft.update(new Vector2d(0, 0), false, 1);
        }
        telemetry.addData("MotorTickArray", driveLeft.motorTicks);
        telemetry.addData("P1 Voltage", driveLeft.potentiometerMap.get("P1").getMV());
        telemetry.addData("P1 Voltage", driveLeft.potentiometerMap.get("P2").getMV());
    }

    @Override
    public void opStart() {
        driveLeft.writeToFile("betterSwerveData.csv");

    }

    @Override
    public void opUpdate(double deltaTime) {
        gamepadHandler.loop(); //update gamepads

        joystickLeft.x = gamepadHandler.value("D1:LS_X");
        joystickLeft.y = -gamepadHandler.value("D1:LS_Y");
        joystickRight.x = gamepadHandler.value("D1:RS_X");
        joystickRight.y = -gamepadHandler.value("D1:RS_Y");

        //I don't use drive.setOutputMultiplier because there is no way I waste CPU cycles on rebuilding the matrix

        driveLeft.update(joystickLeft, false, deltaTime);
        driveRight.update(joystickRight, false, deltaTime);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
