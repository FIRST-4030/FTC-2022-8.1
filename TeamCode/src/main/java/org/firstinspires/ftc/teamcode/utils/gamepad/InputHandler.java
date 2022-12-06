package org.firstinspires.ftc.teamcode.utils.gamepad;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class InputHandler {
    private final HashMap<String, Input> inputs;
    private final OpMode opmode;
    private double lastLoop;

    public InputHandler(OpMode opMode) {
        inputs = new HashMap<>();
        opmode = opMode;
    }

    // Register an input listener
    public void register(String name, GAMEPAD gamepad, PAD_KEY padkey) {
        // Die if we're used poorly
        if (name == null || name.isEmpty() || gamepad == null || padkey == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "Null or empty name, gamepad or padkey");
        }

        // Translate the logical gamepad to a hardware gamepad using our local opmode
        Gamepad hardwareGP = (gamepad == GAMEPAD.driver1 ? opmode.gamepad1 : opmode.gamepad2);

        // Ensure the named button exists, since the compiler can't check
        try {
            Field field = hardwareGP.getClass().getField(padkey.name());
        } catch (NoSuchFieldException e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "Invalid padkey: " + padkey);
        }

        // Warn if this pad-key tuple already exists
        for (Map.Entry<String, Input> k : inputs.entrySet()) {
            Input key = k.getValue();
            if (key.gamepad == hardwareGP && key.padkey == padkey) {
                opmode.telemetry.log().add(getClass().getSimpleName() + ": " +
                        "Duplicate gamepad-padkey: " + gamepad + "-" + padkey);
            }
        }

        // Warn if this key name already exists
        if (inputs.containsKey(name)) {
            opmode.telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Replacing existing key: " + name);
        }

        // Register the key
        inputs.put(name, new Input(hardwareGP, padkey));
    }

    // Remove named button from the handler
    public void deregister(String name) {
        inputs.remove(name);
    }

    // Update all inputs
    public void loop() {
        // Skip processing if we already ran this loop
        if (opmode.time == lastLoop) {
            return;
        }
        lastLoop = opmode.time;

        for (Map.Entry<String, Input> e : inputs.entrySet()) {
            e.getValue().update();
        }
    }

    // Grab the listener for direct access
    @NonNull
    public Input get(String name) {
        Input input = inputs.get(name);

        // Warn and fake an input if this name isn't registered
        // This gives us non-null returns and warnings instead of null pointer exceptions
        if (input == null) {
            opmode.telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Unregistered input: " + name);
            input = new Input(opmode.gamepad2, PAD_KEY.guide);
        }
        return input;
    }

    /*
     * Convenience wrappers
     *
     * Instead of this:
     *     inputs.get("Foo").down();
     * You can use this:
     *     inputs.down("Foo");
     */
    public boolean active(String name) {
        return get(name).active();
    }

    public boolean down(String name) {
        return get(name).down();
    }

    public boolean up(String name) {
        return get(name).up();
    }

    public boolean changed(String name) {
        return get(name).changed();
    }

    public boolean auto(String name) {
        return get(name).auto();
    }

    public boolean toggle(String name) {
        return get(name).toggle();
    }

    public boolean held(String name) {
        return get(name).held();
    }

    public float value(String name) {
        return get(name).value();
    }
}
