package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDExample extends OpMode {
    PID pid;
    DcMotorEx motor;

    @Override
    public void init() {
        pid = new PID(1, 0, 0);
        pid.setTolerance(5);

        motor = hardwareMap.get(DcMotorEx.class, "MOTOR");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        pid.setTarget(100);
    }

    @Override
    public void loop() {
        // PID input from the encoder
        pid.input(motor.getCurrentPosition());

        // PID output back to the motor
        motor.setPower(pid.clamped());

        // Stop when PID says we're done
        if (pid.isDone()) {
            requestOpModeStop();
        }
    }
}
