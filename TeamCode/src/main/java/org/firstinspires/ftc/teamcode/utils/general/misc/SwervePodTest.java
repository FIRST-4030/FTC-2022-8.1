package org.firstinspires.ftc.teamcode.utils.general.misc;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

public class SwervePodTest extends LoopUtil {
    public static InputHandler gamepadHandler;
    public static Vector2d joystick;
    public static SwervePodGeneral drive;


    @Override
    public void opInit() {
        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        joystick = new Vector2d();
        drive = new SwervePodGeneral(hardwareMap, 1, 1);
        drive.mapMotors("Spin", false, "Turn", true, false);

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

        //I don't use drive.setOutputMultiplier because there is no way I waste CPU cycles on rebuilding the matrix

        drive.update(joystick, false, deltaTime);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
