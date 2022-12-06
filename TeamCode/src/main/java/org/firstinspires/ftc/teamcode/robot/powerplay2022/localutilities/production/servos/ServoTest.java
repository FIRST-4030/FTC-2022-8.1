package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Config
@TeleOp(name = "Servo Driver Test", group = "Tester")
public class ServoTest extends LoopUtil {
    public static ServoFTC servo;
    public static ServoConfig config;
    public static Vector2d joyStick = new Vector2d();

    @Override
    public void opInit() {

        config = new ServoConfig("A",false, 0, 1);
        servo = new ServoFTC(hardwareMap, telemetry, config);
        joyStick = new Vector2d(0, 1);


    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        joyStick.x = -gamepad1.right_stick_x;
        joyStick.y = gamepad1.right_stick_y;
        servo.setPosition(Math.atan2(joyStick.y, joyStick.x));
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
