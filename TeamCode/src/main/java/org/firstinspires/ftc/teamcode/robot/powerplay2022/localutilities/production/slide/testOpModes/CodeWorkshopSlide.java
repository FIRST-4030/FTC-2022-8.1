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
        if(gamepad1.x){
            slideLevel = 1;
        }
        if(gamepad1.y){
            slideLevel = 2;
        }
        if(gamepad1.a){
            slideLevel = 3;
        }
        if(gamepad1.b){
            slideLevel = 0;
        }
        slide.update(slideLevel);
        slide.setPower(slide.powerOutput);
    }

}