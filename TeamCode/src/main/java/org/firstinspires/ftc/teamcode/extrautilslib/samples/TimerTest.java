package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.extrautilslib.core.timer.EULClock;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "EUL::TimerTest", group = "EUL")
public class TimerTest extends LoopUtil {

    public static EULClock clock;

    @Override
    public void opInit() {
        clock = new EULClock();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {
        clock.start();
    }

    @Override
    public void opUpdate(double deltaTime) {
        clock.waitFor((long) (3 * EULConstants.SEC2MS));
        telemetry.addData("DeltaTime: ", deltaTime);
        telemetry.addData("ElapsedTime: ", clock.getElapsedTime());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {
        clock.stop();
    }
}
