package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.SlideController;

@TeleOp(name = "CodeWorkshopSlide", group = "CodeWorkshop")
public class CodeWorkshopSlide extends OpMode {
    SlideController slide;
    Gamepad userInterface;

    int slideLevel;
    double slidePower;

    @Override
    public void init() {
        slide = new SlideController(hardwareMap, "LSRM", false);
        userInterface = gamepad1;
        slideLevel = 0;
        slidePower = 1;
    }

    @Override
    public void loop() {
        if (userInterface.x){
            slideLevel = 1;
        }

        if (userInterface.y){
            slideLevel = 2;
        }
        if (userInterface.b){
            slideLevel = 3;
        }

        if (userInterface.a){
            slideLevel = 0;
        }

        slide.update(slideLevel);
        slide.setPower(slide.powerOutput);
    }
}
