package org.firstinspires.ftc.teamcode.utils.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

public class Input {
    // Config
    private static final float AXIS_THRESHOLD = 0.4f;
    private static final int AUTOKEY_DEFAULT = 50;
    private static final int HELD_DEFAULT = 400;

    // Definition
    public final Gamepad gamepad;
    public final PAD_KEY padkey;

    // Members
    private int autoDelay = AUTOKEY_DEFAULT;
    private int heldDelay = HELD_DEFAULT;
    private long autoTime = 0;
    private long heldTime = 0;
    private boolean auto = false;
    private boolean toggle = false;
    private boolean held = false;
    private boolean heldFired = false;

    // Raw state
    private boolean active;
    private float value;
    private boolean lastActive;
    private float lastValue;

    public Input(Gamepad gamepad, PAD_KEY key) {
        this.gamepad = gamepad;
        this.padkey = key;
    }

    // Grab the input state
    public void update() {
        boolean a;
        float v;
        try {
            Field field = gamepad.getClass().getField(padkey.name());
            if (boolean.class.isAssignableFrom((field.getType()))) {
                a = field.getBoolean(gamepad);
                v = a ? 1.0f : 0f;
            } else {
                v = field.getFloat(gamepad);
                a = (Math.abs(v) >= AXIS_THRESHOLD);
            }
        } catch (Exception e) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "Unable to read input: " + gamepad + "-" + padkey);
        }

        // Rotate and store the raw state
        lastValue = value;
        value = v;
        lastActive = active;
        active = a;

        // Consolidate system calls
        long now = System.currentTimeMillis();

        // Toggle flips on key up
        if (up()) {
            toggle = !toggle;
        }

        // Auto fires on key down() and every autoDelay milliseconds until release
        auto = (down() || (active && now > autoTime));
        if (auto) {
            autoTime = now + autoDelay;
        }

        // Held fires heldDelay ms after down(), if nothing has changed since then
        held = (!heldFired && (active && !down()) && now > heldTime);
        if (held) {
            heldFired = true;
        } else if (up()) {
            heldFired = false;
        }
        if (down()) {
            heldTime = now + heldDelay;
        }
    }

    /**
     * Active fires from key down until key up
     */
    public boolean active() {
        return active;
    }

    /**
     * Down fires only on key down
     */
    public boolean down() {
        return (!lastActive && active);
    }

    /**
     * Up fires only on key up
     */
    public boolean up() {
        return (lastActive && !active);
    }
    /**
     * Changed fires when the boolean or float value changes
     */
    public boolean changed() {
        return (active != lastActive || lastValue != value);
    }

    /**
     * Auto fires on key down() and every autoDelay milliseconds until release
     */
    public boolean auto() {
        return auto;
    }

    /**
     * Toggle fires after a key down/up cycle until the next down/up cycle
     */
    public boolean toggle() {
        return toggle;
    }

    /**
     * Held fires heldDelay ms after down(), if nothing has changed since then
     */
    public boolean held() {
        return held;
    }

    /**
     * Value returns the analog value of this input, or 0/1 for digital inputs
     */
    public float value() {
        return value;
    }

    // Set the delay between autokey repeats
    public void setAutoDelay(int timeout) {
        autoDelay = timeout;
    }

    // Set the delay before held() fires
    public void setHeldDelay(int timeout) {
        heldDelay = timeout;
    }

    // Force toggle into a particular state
    public void setToggle(boolean active) {
        toggle = active;
    }
}
