package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions.Conditional;

@TeleOp(name = "PowerPlayLinearSlidePIDTest", group = "Testers")
public class PowerPlayLinearSlidePIDTest extends EnhancedOpMode {

    private LinearSlide.PowerPlayLS linearSlide;

    @Override
    public void opInit() {
        linearSlide = new LinearSlide.PowerPlayLS(hardwareMap, "LSRM", false);
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
        if (driver1.dPadDown.released) linearSlide.setPosition(0);
        if (driver1.dPadLeft.released) linearSlide.setPosition(1);
        if (driver1.dPadUp.released) linearSlide.setPosition(2);
        if (driver1.dPadRight.released) linearSlide.setPosition(3);
    }

    @Override
    public void opUpdate(double deltaTimeMS) {

    }

    @Override
    public void opFixedUpdate(double deltaTimeMS) {

    }

    @Override
    public void opStop() {

    }
}
