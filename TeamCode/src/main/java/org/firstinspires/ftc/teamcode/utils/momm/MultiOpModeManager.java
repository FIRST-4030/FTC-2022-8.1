package org.firstinspires.ftc.teamcode.utils.momm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;

import java.util.Vector;

public class MultiOpModeManager extends OpMode {

    private final Vector<OpMode> opmodes;
    public final InputHandler input;

    // Init the list and Globals
    public MultiOpModeManager() {
        Globals.opmode = this;
        opmodes = new Vector<>();
        input = new InputHandler(this);
    }

    // Add new OMs for concurrent execution
    public void register(OpMode opMode) {
        // Validate
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        if (opMode.equals(this)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "Refusing to re-register the primary OpMode: " + opMode.getClass().getSimpleName());
        }
        if (opmodes.contains(opMode)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "OpMode already registered: " + opMode.getClass().getSimpleName());
        }

        // One-time OpMode setup for child opmodes
        // This lets the built-in vars work normally in both stand-alone and MOMM modes
        opMode.hardwareMap = hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = gamepad1;
        opMode.gamepad2 = gamepad2;

        // Enable
        opmodes.add(opMode);
    }

    // Remove registered OMs
    public void deregister(OpMode opMode) {
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        opmodes.remove(opMode);
    }

    /*
     * Run each of the standard OM methods for each registered OM
     */
    @Override
    public void init() {
        for (OpMode om : opmodes) {
            om.time = time; // Time changes each cycle
            om.init();
        }
    }

    @Override
    public void init_loop() {
        input.loop();
        for (OpMode om : opmodes) {
            om.time = time;
            om.init_loop();
        }
    }

    @Override
    public void start() {
        for (OpMode om : opmodes) {
            om.time = time;
            om.start();
        }
    }

    @Override
    public void loop() {
        input.loop();
        for (OpMode om : opmodes) {
            om.time = time;
            om.loop();
        }
    }

    @Override
    public void stop() {
        for (OpMode om : opmodes) {
            om.time = time;
            om.stop();
        }
    }
}