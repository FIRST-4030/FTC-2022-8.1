package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class PID {
    /* Dashboard members */
    public static boolean LOG = false;

    /* Public members */
    public double tolerance = 5.0;
    public double target;
    public double P, I, D;

    /* Internal state */
    private final ElapsedTime interval;
    private PIDState current, last;

    public PID() {
        this(1.0, 0.0, 0.0);
    }

    public PID(double p, double i, double d) {
        P = p;
        I = i;
        D = d;
        target = 0;
        current = last = null;
        interval = new ElapsedTime(0);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean isDone() {
        // We're not done if we have never had input
        if (current == null) {
            return false;
        }

        // We are done if the current error is with tolerance of the target
        return (Math.abs(current.error) <= tolerance);
    }

    public void input(double actual) {
        PIDState state = new PIDState();

        // If we don't have a previous state use the new one we're building
        // This makes calls to last.* safe in the later math
        if (last == null) {
            last = current = state;
        }

        // A little math to calculate the new state
        state.target = target;
        state.actual = actual;
        state.relative = state.actual - last.actual;
        state.error = state.target - state.actual;
        state.accumulatedError = last.accumulatedError + state.error;
        state.differentialError = state.error - last.error;
        state.interval = interval.seconds();

        // Log new samples
        if (LOG) {
            RobotLog.d("PID_UPDATE," + state.csv());
        }

        // Swap in the new state and reset the interval timer
        last = current;
        current = state;
        interval.reset();
    }

    public double output() {
        // Don't run if we haven't had input
        if (current == null) {
            return 0;
        }

        /* The definitional PID is:
         * P * current error
         * I * accumulated error
         * D * error slope (i.e. Δerror / Δtime)
         *
         * The standard formulation is:
         *     output = P*ce + I*ae + D*(Δe/Δt)
         * A typical formulation for discrete samples is:
         *     output = P * (ce + (I/Δt)*(ae) + (D/Δt)*(Δe/Δt))
         * which applies P as a scaling factor to I and D, so that P provides overall gain control
         *
         * For discussion of PID formulas see:
         * https://www.sentekdynamics.com/sentek-dynamics-news/2020/8/24/pid-control-theory
         *
         * // Standard form:
         * double output =  (P * current.error) +
         *         (I * current.accumulatedError) +
         *         (D * (current.differentialError / current.interval));
         */

        // P-scaled form:
        double output = P * (
                (current.error) +
                        ((1 / I) * (current.accumulatedError * current.interval)) +
                        (D * (current.differentialError / current.interval))
        );

        // Log internals at time of use
        if (LOG) {
            Locale l = Locale.US;
            String s = "PID_OUTPUT," +
                    "output," + String.format(l, "%.04f", output) + "," +
                    "P," + String.format(l, "%.04f", P) + "," +
                    "I," + String.format(l, "%.04f", I) + "," +
                    "D," + String.format(l, "%.04f", D) + "," +
                    current.csv();
            RobotLog.d(s);
        }

        return output;
    }

    /* Same as output() but clamped to the range -1.0 to 1.0 */
    public double clamped() {
        return Math.max(-1, Math.min(1, output()));
    }

    public class PIDState {
        double target;
        double actual;
        double relative;
        double error;
        double accumulatedError;
        double differentialError;
        double interval;

        public String csv() {
            Locale l = Locale.US;
            final String s =
                    "target," + String.format(l, "%.04f", target) + "," +
                            "relative," + String.format(l, "%.04f", relative) + "," +
                            "error," + String.format(l, "%.04f", error) + "," +
                            "accumulatedError," + String.format(l, "%.04f", accumulatedError) + "," +
                            "differentialError," + String.format(l, "%.04f", differentialError) + "," +
                            "interval," + String.format(l, "%.04f", interval);
            return s;
        }
    }
}
