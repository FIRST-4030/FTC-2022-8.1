package org.firstinspires.ftc.teamcode.drives.driveto;

import java.security.InvalidParameterException;

public class PIDParams {
    public float P;
    public float I;
    public float D;

    public int min = 0;
    public int max = 0;

    // If set, limit the accumulator value to the range ±maxAccumulator
    public Float maxAccumulator;
    // If true, reset the accumulated error whenever the error sign changes
    // This is useful when the input() values are not rate-based (e.g. location displacement or heading)
    public boolean resetAccumulatorOnErrorSignChange;
    // If true, reset the accumulator error whenever the target sign changes
    // This is usually desirable but might interfere with non-rate, non-normalized input() values
    public boolean resetAccumulatorOnTargetSignChange;

    public PIDParams() {
        this(0.1f, 0.01f, 0.0f);
    }

    public PIDParams(float p, float i, float d) {
        this(p, i, d, 0, 1);
    }

    public PIDParams(float p, float i, float d, int min, int max) {
        this(p, i, d, min, max,
                null,
                false,
                true);
    }

    public PIDParams(float p, float i, float d,
                     Float maxAccumulator,
                     boolean resetAccumulatorOnErrorSignChange,
                     boolean resetAccumulatorOnTargetSignChange) {
        this(p, i, d, 0, 1,
                maxAccumulator,
                resetAccumulatorOnErrorSignChange,
                resetAccumulatorOnTargetSignChange);
    }

    public PIDParams(float p, float i, float d, int min, int max,
                     Float maxAccumulator,
                     boolean resetAccumulatorOnErrorSignChange,
                     boolean resetAccumulatorOnTargetSignChange) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.min = min;
        this.max = max;
        this.maxAccumulator = maxAccumulator;
        this.resetAccumulatorOnErrorSignChange = resetAccumulatorOnErrorSignChange;
        this.resetAccumulatorOnTargetSignChange = resetAccumulatorOnTargetSignChange;

        if (min >= max) {
            throw new InvalidParameterException(this.getClass().getSimpleName() + ": Invalid min/max");
        }

    }
}
