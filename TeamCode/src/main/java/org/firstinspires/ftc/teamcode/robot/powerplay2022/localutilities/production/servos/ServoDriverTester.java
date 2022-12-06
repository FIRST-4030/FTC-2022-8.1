package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.customDriver.Custom360Servo;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Config
@Disabled
@TeleOp(name = "Servo Custom Driver Tester v2", group = "Tester")
public class ServoDriverTester extends LoopUtil {
    public static ServoConfig config;
    public static Custom360Servo servo;

    @Override
    public void opInit() {
        config = new ServoConfig("R", false, 0, 1);

        servo = new Custom360Servo(hardwareMap, telemetry, config, 40, 60);

    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        servo.setAngle(Math.PI/2);
        handleTelemetry();
    }

    public void handleTelemetry(){

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
