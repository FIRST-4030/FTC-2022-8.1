package org.firstinspires.ftc.teamcode.utils.momm.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MOMM_Speech extends OpMode {
    // Constants
    public static double SPEECH_DELAY = 5.0;

    // Members
    private String text = null;
    private ElapsedTime delay = new ElapsedTime();

    // External control
    public void speak(String t) {
        text = t;
    }

    // Standard methods
    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        // Use om.time, in case we aren't the primary OpMode
        telemetry.addData("Init Time", "%.2f", time);
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
    }

    @Override
    public void loop() {
        if (text != null && !text.isEmpty() && delay.seconds() > SPEECH_DELAY) {
            telemetry.speak(text);
            text = null;
            delay.reset();
        }
        telemetry.addData("Loop Time", "%.2f", time);
    }
}
