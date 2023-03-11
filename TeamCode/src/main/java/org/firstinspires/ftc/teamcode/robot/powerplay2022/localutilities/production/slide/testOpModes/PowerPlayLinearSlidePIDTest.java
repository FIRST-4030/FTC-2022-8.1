package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode.EnhancedOpMode;

@TeleOp(name = "PowerPlayLinearSlidePIDTest", group = "Testers")
public class PowerPlayLinearSlidePIDTest extends EnhancedOpMode {

    private LinearSlide.PowerPlayLS linearSlide;
    private int slidePosition;

    @Override
    public void opInit() {
        this.slidePosition = 0;
        this.linearSlide = new LinearSlide.PowerPlayLS(hardwareMap, "LSRM", false);
    }

    @Override
    public void opSetupControllers(DSController driver1, DSController driver2) {

    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opInput(DSController driver1, DSController driver2, double deltaTimeMS) {
        if (driver1.dPadDown.released) slidePosition = 0;
        if (driver1.dPadLeft.released) slidePosition = 1;
        if (driver1.dPadUp.released) slidePosition = 2;
        if (driver1.dPadRight.released) slidePosition = 3;
    }

    @Override
    public void opUpdate(double deltaTimeMS) {
        linearSlide.setPosition(slidePosition);
    }

    @Override
    public void opFixedUpdate(double deltaTimeMS) {

    }

    @Override
    public void opStop() {

    }
}
