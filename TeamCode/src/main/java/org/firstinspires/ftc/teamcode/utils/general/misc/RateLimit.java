package org.firstinspires.ftc.teamcode.utils.general.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RateLimit {
    private OpMode opMode;
    private double lastTime;
    private double last;
    public double maxRate;

    public RateLimit(OpMode opMode, double maxRate) {
        if (opMode == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null opmode");
        }
        this.opMode = opMode;
        setMaxRate(maxRate);
    }

    public void setMaxRate(double maxRate) {
        if (maxRate == 0) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Rate must not be zero");
        }
        this.maxRate = maxRate;
    }

    public double update(double delta) {
        double deltaT = opMode.time - lastTime;
        if (Math.abs(delta) > maxRate / deltaT) {
            last = Math.copySign(maxRate * deltaT, delta);
        } else {
            last = delta;
        }
        lastTime = opMode.time;
        return last;
    }
}
