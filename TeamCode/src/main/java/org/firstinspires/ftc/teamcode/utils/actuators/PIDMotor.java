package org.firstinspires.ftc.teamcode.utils.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drives.driveto.PID;
import org.firstinspires.ftc.teamcode.utils.sensors.switches.Switch;
import org.firstinspires.ftc.teamcode.utils.threadingUtils.Background;

public class PIDMotor extends Motor {
    private static final boolean DEBUG = false;
    private static final int TIMEOUT = 3000;

    private Background background;
    private final PIDMotorConfig config;
    private boolean initialized;
    private long timeout = 0;
    public final PID pid; // sorry private variable!!!

    public PIDMotor(HardwareMap map, Telemetry telemetry, PIDMotorConfig config) {
        super(map, telemetry, config);
        this.config = config;
        initialized = false;
        pid = new PID(config.pid);
    }

    public void init() {
        setInitialized();
        pid.setTarget(getEncoder());
        start();
    }

    // Generic implementation for a switch-based reset, can be overridden
    public boolean init(Switch button, float speed) {
        // Don't re-init unless someone clears initialized
        if (pidAvailable()) {
            return true;
        }

        // Timeout for safety
        if (timeout == 0) {
            timeout = System.currentTimeMillis() + TIMEOUT;
        }
        if (System.currentTimeMillis() > timeout) {
            stop();
            telemetry.log().add(this.getClass().getSimpleName() + ": Init timeout: " + config.name);
            return false;
        }

        // If we're not at 0, run as directed
        if (!button.get()) {
            motor.setPower(speed);
            return false;
        }

        // Stop and set our zero point
        resetEncoder();
        stop();

        // Finalized
        init();
        return true;
    }

    private void loop() {
        if (!isAvailable()) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Cannot loop: " + config.name);
            return;
        }

        pid.input(getEncoder());
        motor.setPower(pid.output());
        if (DEBUG) {
            telemetry.log().add("A/T/P: " + getEncoder() + "\t" + pid.target + "\t" + pid.output());
        }
    }

    public void setInitialized() {
        initialized = true;
    }

    public void set(int target) {
        if (!isAvailable()) return;

        target = Math.max(target, config.min);
        target = Math.min(target, config.max);
        pid.setTarget(target);
    }

    public boolean pidAvailable() {
        return super.isAvailable() && initialized;
    }

    public void start() {
        if (!pidAvailable()) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Cannot start: " + config.name);
        }

        final PIDMotor me = this;
        background = new Background() {
            @Override
            public void loop() {
                me.loop();
            }
        };
        background.start();
    }

    public void stop() {
        if (background != null) {
            background.stop();
        }
    }
}
