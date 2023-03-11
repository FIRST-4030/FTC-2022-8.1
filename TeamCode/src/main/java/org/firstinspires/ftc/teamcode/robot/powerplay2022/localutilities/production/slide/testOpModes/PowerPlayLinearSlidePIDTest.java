package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions.Conditional;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "PowerPlayLinearSlidePIDTest", group = "Testers")
public class PowerPlayLinearSlidePIDTest extends LoopUtil {

    private LinearSlide.PowerPlayLS linearSlide;

    @Override
    public void opInit() {
        linearSlide = new LinearSlide.PowerPlayLS(hardwareMap, "LSRM", false);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

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
