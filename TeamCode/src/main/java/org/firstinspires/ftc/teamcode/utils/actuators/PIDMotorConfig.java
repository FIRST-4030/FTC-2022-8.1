package org.firstinspires.ftc.teamcode.utils.actuators;

import org.firstinspires.ftc.teamcode.drives.driveto.PIDParams;

import java.security.InvalidParameterException;

public class PIDMotorConfig extends MotorConfig {
    public final PIDParams pid;
    public final int max;
    public final int min;

    public PIDMotorConfig(MotorConfig config, PIDParams pid, int min, int max) {
        super(config);
        this.pid = pid;
        this.max = max;
        this.min = min;

        if (min >= max) {
            throw new InvalidParameterException(this.getClass().getSimpleName() + ": Invalid min/max: " + config.name);
        }
    }
}
