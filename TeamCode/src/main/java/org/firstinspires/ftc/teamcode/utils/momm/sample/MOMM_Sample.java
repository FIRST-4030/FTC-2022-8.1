package org.firstinspires.ftc.teamcode.utils.momm.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;

// Extend MultiOpModeManager instead of OpMode
// Register with @TeleOp or @Autonomous as with any other OpMode
@TeleOp(name = "MOMM_Sample", group = "MOMM")
@Disabled
public class MOMM_Sample extends MultiOpModeManager {
    // External OMs
    // OMs that you will call directly should have members
    // OMs that are complete independent can be defined in-line (see init())
    private MOMM_Speech speech;

    /*
     * Standard OM methods
     *
     * All of the standard OM methods are available to @override
     * It is acceptable to exclude a method; for example, if init_loop is empty it can be excluded
     * If you define one of the standard methods it must call the same method's super()
     * super() can be called anywhere within the method; first is often best
     */

    @Override
    public void init() {
        // Register the Drive and Speech OMs
        super.register(new MOMM_Drive());

        // Register the speech OM
        speech = new MOMM_Speech();
        super.register(speech);

        // Register a button for testing
        input.register("SPEAK", GAMEPAD.driver1, PAD_KEY.a);

        // Be sure to register OMs before this line or they won't get an init() call
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        // Speak when requested
        if (input.down("SPEAK")) {
            speech.speak("Time: " + (int) time);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}