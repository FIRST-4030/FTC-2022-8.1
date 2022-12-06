/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;
import org.firstinspires.ftc.teamcode.utils.general.maths.piecewise.PiecewiseFunction;

@Config
@Disabled
@TeleOp(name = "DuckSpin", group = "Test")
public class DuckSpin extends OpMode {
    // Hardware
    private DcMotor duck = null;
    private DcMotor duck2 = null;

    // Config
    public static double teleopMin = 0.66;
    public static double teleopMax = 0.78;
    public static double teleopRamp = 1.46;
    public static double teleopRampStop = 1;
    public static double autoMin = 0.45;
    public static double autoMax = 0.6;
    public static double autoRamp = 2.4;
    public static double autoRampStop = 1.525;

    public static double pow = 3;
    private boolean started;
    private boolean done;

    // Members
    public static boolean DEBUG = false;
    private boolean enabled = false;
    private double speed = 0.0;
    public static double speedMin = teleopMin;  // min duck spinner speed (0 - 1.0)
    public static double speedMax = teleopMax;  // max duck spinner speed (0 - 1.0)
    public static double rampTime = teleopRamp;  // duck spinner ramp time (seconds, >0)
    public static double rampStop = teleopRampStop;
    private final ElapsedTime timer = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.IDLE;
    private boolean auto = false;
    private boolean redSide = false;

    @Override
    public void init() {
        // Duck spinner
        try {
            duck = hardwareMap.get(DcMotor.class, "duck");
            duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            duck2 = hardwareMap.get(DcMotor.class, "duck2");
            duck2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            duck2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    // sets speeds and time to teleop values
    public void teleop(boolean red) {
        speedMin = teleopMin;
        speedMax = teleopMax;
        rampTime = teleopRamp;
        rampStop = teleopRampStop;
        redSide = red;
        auto = false;
        state = red ? AUTO_STATE.SPIN_RED : AUTO_STATE.SPIN_BLUE;
    }

    // sets speeds and time to autonomous values
    public void auto(boolean red) {
        speedMin = autoMin;
        speedMax = autoMax;
        rampTime = autoRamp;
        rampStop = autoRampStop;
        auto = true;
        if (state == AUTO_STATE.IDLE) {
            redSide = red;
            state = red ? AUTO_STATE.SPIN_RED : AUTO_STATE.SPIN_BLUE;
        }
    }

    public boolean isDone() {
        return (state == AUTO_STATE.DONE);
    }

    @Override
    public void loop() {
        // Override the current auto state with driver commands
        // Technically we should only trigger on button-down, not repeatedly while held
        if (gamepad1.a) {
            teleop(false);
        }
        if (gamepad1.b) {
            teleop(true);
        }

        // Run the auto cycle (including translated driver commands)
        switch (state) {
            case IDLE:
                break;
            case SPIN_BLUE:
                // Run the spin cycle
                duckRampPoly(speedMin, speedMax, rampTime, rampStop, pow, auto, redSide);
                if (done && duck.getPower() == 0 && duck2.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case SPIN_RED:
                // Run the spin cycle
                duckRampPoly(-speedMin, -speedMax, rampTime, rampStop, pow, auto, redSide);
                if (done && duck.getPower() == 0 && duck2.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case DONE:
                duck.setPower(0);
                duck2.setPower(0);
                break;
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Duck State", state);
            telemetry.addData("Duck Output", "%.2f/%d",
                    duck.getPower(), duck.getCurrentPosition());
        }

        telemetry.addData("speedMin:", teleopMin);
        telemetry.addData("speedMax:", teleopMax);
        /*
         * TODO: This is a great place to use the InputHandler.auto() method
         ** It's specifically meant for this sort of incremental manual control
         ** auto() lets this bit of code trigger repeatedly while the key is held
         ** but also limits the repeat rate so it doesn't go too fast
         ** if (in.auto("SPEED_UP")) {
         **   speedMax = Math.min(1.0f, speedMax + 0.01);
         **   speedMin = Math.min(1.0f, speedMin + 0.01);
         ** }
         ** You can adjust the key-repeat rate with a call like this:
         ** in.get("SPEED_UP").setAutoDelay(50);
         *
         * You should also be able to adjust speedMax/speedMin directly in the dashboard
         * Tuning controls on the gamepad can be useful, but the dashboard requires less code
         */
    }

    @Override
    public void stop() {
    }

    // Duck Ramp Speed Function
    // speedMin is starting speed, -1 to 1
    // speedMax is speed before accelerating to throw duck, -1 to 1
    // time is the total time of one routine
    // rampStop is the time before it accelerates to full speed
    // pow is the power for the poly curve
    // boolean auto determines if it's autonomous
    // boolean red determines if it is red side or blue side
    public void duckRampPoly(double speedMin, double speedMax, double time, double rampStop, double pow, boolean auto, boolean red) {
        if (time == 0) {
            return;
        }

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);
        double y;

        double x = timer.seconds();
        if (timer.seconds() <= rampStop) {
            y = Math.pow(rampStop, -pow) * (speedMax - speedMin) * Math.pow(x, pow) + speedMin;
        } else if (timer.seconds() <= time) {
            y = ((Math.signum(speedMax) * 1) - speedMax) / (time - rampStop) * (timer.seconds() - rampStop) + speedMax;
        } else {
            y = 0;
        }

        if (!done && started) {
            if (!auto) {
                // speed is calculated using the curve defined above
                duck.setPower(y);
                duck2.setPower(y);
            } else {
                if (red) {
                    duck.setPower(y);
                } else {
                    duck2.setPower(y);
                }
            }
            done = (timer.seconds() > time);
        }

        if (!started) {
            timer.reset();
            started = true;
            done = false;
        } else if (done) {
            duck.setPower(0);
            duck2.setPower(0);
            started = false;
        }
    }

    // An encoder-synchronized piecewise function for unloading ducks quickly
    // Not currently used and needs improvements/further steps
    private int rampBarThrow(PiecewiseFunction pfunc, int currentTicks) {
        // A speed slow enough for a safe start
        double speedStart = teleopMin;
        // The maximum speed at which ducks can safely travel
        double speedMax = teleopMax; // TODO: Calculate from measurements
        // The best speed for removing ducks from the spinner
        double speedEject = 1;

        // Minimum number of ticks until it's safe to run at speedMax
        int startTicks = 500; // TODO: Approximate the best observed ramp rate
        // Maximum number of ticks from duck-start to duck-on-bar
        // Spinner will move as quickly as ramp limits allow into this position
        int barTicks = 15000; // TODO: Measure travel distance in ticks from start to bar
        // Maximum number of ticks from duck-on-bar to duck-ejected
        // Spinner will move this additional distance at ejection speeds
        int ejectTicks = 5000; // TODO: Estimate the overrun needed to eject

        pfunc.addElement(currentTicks, speedStart);
        pfunc.addElement(currentTicks + startTicks, speedMax);
        pfunc.addElement(currentTicks + barTicks - 1, speedMax);
        pfunc.addElement(currentTicks + barTicks, speedEject);
        pfunc.addElement(currentTicks + barTicks + ejectTicks - 1, speedEject);
        pfunc.addElement(currentTicks + barTicks + ejectTicks, 0);

        // Enable first/last element clamping in case the encoder values drift outside the model
        pfunc.setClampLimits(true);

        // Return the cycle-complete tick count, for upstream progress monitoring
        return currentTicks + barTicks + ejectTicks;
    }

    // enum list of auto states
    enum AUTO_STATE implements OrderedEnum {
        IDLE,
        SPIN_BLUE,
        SPIN_RED,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}