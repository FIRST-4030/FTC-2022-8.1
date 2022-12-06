package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;

public class Globals {
    static public OpMode opmode = null;
    static public InputHandler input = null;

    // Callers must provide an OpMode, so this will never fail to return an OpMode
    // Makes it easy to use in one-liners that may or may not be the parent OpMode
    static public OpMode opmode(OpMode om) {
        // Ensure the new OpMode exists (though it may not be active or useful)
        if (om == null) {
            throw new IllegalArgumentException(Globals.class.getSimpleName() + ":" +
                    "Invalid OpMode provided");
        }
        // Take any OpMode if we have none
        if (opmode == null) {
            opmode = om;
        }
        // Globals.opmode is now guaranteed to be non-null

        // Prefer the new OpMode if it has a gamepad that doesn't match the current OpMode
        // TODO: Not sure if this will check detect the right state but we'll give it a shot
        // TODO: We may be able to check hardwareMap if this doesn't work
        // TODO: We may need to wrap the check as try/catch to a reflected method call that fails
        if (om.gamepad1 != null && opmode.gamepad1 != null &&
                !opmode.gamepad1.equals(om.gamepad1)) {
            opmode = om;
        }

        // Health checks for troubleshooting, visible in logcat on the robot controller device
        // Ideally these are never true, but if they are knowing would help
        // TODO: This should maybe throw an exception? But I want to test before breaking things
        if (opmode.hardwareMap == null || opmode.telemetry == null || opmode.gamepad1 == null) {
            System.err.println(Globals.class.getSimpleName() + ":" +
                    "OpMode is unhealthy");
        }
        return opmode;
    }

    static public InputHandler input(OpMode om) {
        if (input == null) {
            input = new InputHandler(opmode(om));
        }
        return input;
    }
}
