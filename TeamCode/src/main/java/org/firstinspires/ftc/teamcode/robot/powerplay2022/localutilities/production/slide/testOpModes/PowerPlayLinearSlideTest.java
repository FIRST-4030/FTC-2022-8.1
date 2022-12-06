package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Disabled
@TeleOp(name = "PowerPlayLinearSlideTest", group = "Testers")
public class PowerPlayLinearSlideTest extends LoopUtil {

    public InputHandler inputHandler;
    public SlideController controller;

    public DcMotor left, right;

    public double linearSlideSpeed = 0.5;
    public SlideController.LEVEL slideLevel = SlideController.LEVEL.REST;
    public double lsInput = 0;

    @Override
    public void opInit() {
        inputHandler = InputAutoMapper.normal.autoMap(this);
        controller = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);

        left = controller.getLeft();
        right = controller.getRight();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        handleInput(deltaTime);
        controller.update(deltaTime, slideLevel, linearSlideSpeed);
        outputTelemetry();
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }

    public void handleInput(double deltaTime){
        inputHandler.loop();
        if (gamepad1.a){
            slideLevel = SlideController.LEVEL.REST;
        } else if (gamepad1.b){
            slideLevel = SlideController.LEVEL.LOW;
        } else if (gamepad1.y){
            slideLevel = SlideController.LEVEL.MIDDLE;
        } else if (gamepad1.x){
            slideLevel = SlideController.LEVEL.HIGH;
        }
    }

    public void outputTelemetry(){
        telemetry.addData("Left Encoder Ticks: ", left.getCurrentPosition());
        telemetry.addData("Right Encoder Ticks: ", right.getCurrentPosition());
    }
}
