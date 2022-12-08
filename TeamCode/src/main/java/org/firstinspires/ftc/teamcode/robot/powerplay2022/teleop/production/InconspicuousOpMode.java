package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.pathfinder.control.mecanum.PathFinderMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "XD", group = "Z")
public class InconspicuousOpMode extends LoopUtil {

    public static InputHandler inputHandler;
    public static PathFinderMecanumDrive mecanumDrive;
    public Vector3d joystickVector;

    @Override
    public void opInit() {
        inputHandler = InputAutoMapper.normal.autoMap(this);
        joystickVector = new Vector3d();

        mecanumDrive = new PathFinderMecanumDrive(hardwareMap, telemetry,
                "FL", false,
                "FR", false,
                "BL", false,
                "BR", false);

        mecanumDrive.setDriveToVelocityMode();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        mecanumDrive.joystickControl(joystickVector, false);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }

    public void handleInput(){
        inputHandler.loop();

        joystickVector.x = gamepad1.left_stick_x;
        joystickVector.y = -gamepad1.left_stick_y;
        joystickVector.z = gamepad1.right_stick_x;
    }

    public void handleTelemetry(){
        telemetry.addData("Joystick: ",
                  "\nADVANCE: " + joystickVector.x +
                        "\nSTRAFE: " + joystickVector.y +
                        "\nTURN: " + joystickVector.z);
        mecanumDrive.handleTelemetry(telemetry);
    }
}
