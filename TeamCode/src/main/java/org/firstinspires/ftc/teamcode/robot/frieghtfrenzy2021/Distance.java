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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@TeleOp(name = "Distance", group = "Test")
public class Distance extends OpMode {
    // Hardware
    private DistanceSensor left = null;
    private DistanceSensor right = null;

    // Config
    public static boolean DEBUG = false;
    public static int DISTANCE = 19;
    public static double SENSOR_TIME = 0.5;

    // Members
    private boolean enabled = false;
    private ElapsedTime timer = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.IDLE;
    private BARCODE position = BARCODE.NONE;
    private ElapsedTime age = new ElapsedTime(0); // Start=0 to make age start high

    // Intermediates
    private double leftLast = 0;
    private double rightLast = 0;
    private double leftAccum = 0;
    private double rightAccum = 0;
    private boolean leftinRange = false;
    private boolean rightinRange = false;

    @Override
    public void init() {
        // Distance
        try {
            left = hardwareMap.get(DistanceSensor.class, "DL");
            right = hardwareMap.get(DistanceSensor.class, "DR");

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }

        // Known initial conditions
        clear();
        reset();
    }

    @Override
    public void init_loop() {
        // Start sampling as soon as we've got hardware
        if (enabled) {
            loop();
        }
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        telemetry.addData("Position: ", position);
        telemetry.addData("Left in Range: ", leftinRange);
        telemetry.addData("Right in Range: ", rightinRange);

        // Step through the auto commands
        switch (state) {
            case IDLE:
                // Never advance
                break;
            case START_LEFT: // This works like (START_LEFT || START_RIGHT)
            case START_RIGHT:
                timer.reset();
                // This will advance to SAMPLE_LEFT or SAMPLE_RIGHT automatically
                // State order is defined by position the Enum, not by the position here in switch()
                state = state.next();
                break;
            case SAMPLE_LEFT: // Could merge with SAMPLE_RIGHT
                leftLast = left.getDistance(DistanceUnit.INCH);
                leftAccum = (leftAccum * 0.9) + (0.1 * leftLast);
                if (timer.seconds() > SENSOR_TIME) {
                    state = state.next();
                }
                break;
            case PARSE_LEFT: // Could merge with PARSE_RIGHT
                leftinRange = (leftAccum <= DISTANCE);
                state = state.next();
                break;
            case SAMPLE_RIGHT:
                rightLast = right.getDistance(DistanceUnit.INCH);
                rightAccum = (rightAccum * 0.9) + (0.1 * rightLast);
                if (timer.seconds() > SENSOR_TIME) {
                    state = state.next();
                }
                break;
            case PARSE_RIGHT:
                rightinRange = (rightAccum <= DISTANCE);
                state = state.next();
                break;
            case CALCULATE:
                // Be sure this is done atomically
                // We want the reading transitions directly from one valid state the next
                if (leftinRange && !rightinRange) {
                    position = BARCODE.LEFT;
                } else if (!leftinRange && rightinRange) {
                    position = BARCODE.RIGHT;
                } else if (!leftinRange && !rightinRange) {
                    position = BARCODE.CENTER;
                } else {
                    position = BARCODE.NONE;
                }
                age.reset();
                state = state.next();
                break;
            case DONE:
                break;
        }

        telemetry.addData("Left Distance(inches): ", left.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance(inches): ", right.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance(mm): ", left.getDistance(DistanceUnit.MM));
        telemetry.addData("Right Distance(mm): ", right.getDistance(DistanceUnit.MM));

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Distance State", state);
            telemetry.addData("Distance Input", "L %s/%.0f/%.0f, R %s/%.0f/%.0f",
                    leftinRange ? "+" : "-", leftLast, leftAccum,
                    rightinRange ? "+" : "-", rightLast, rightAccum);
            telemetry.addData("Distance Output", position);
        }
    }

    @Override
    public void stop() {
    }

    // Fire off a scan sequence
    public void startScan() {
        reset();
        state = AUTO_STATE.START_LEFT;
    }

    public double leftDistanceMM() {
        return left.getDistance(DistanceUnit.MM);
    }

    public double rightDistanceMM() {
        return right.getDistance(DistanceUnit.MM);
    }

    // Export the result, including "no valid result"
    public BARCODE position() {
        return position;
    }

    // Export the age of the last valid reading
    public double age() {
        return age.seconds();
    }

    // Clear the last scan result, if any
    // Keeping this separate from reset() allows the last reading
    // to remain available while the next scan runs
    public void clear() {
        position = BARCODE.NONE;
        age = new ElapsedTime(0);
    }

    // Return the state machine to starting conditions to allow a new scan
    public void reset() {
        state = AUTO_STATE.IDLE;
        leftLast = 0;
        rightLast = 0;
        leftAccum = 0;
        rightAccum = 0;
        leftinRange = false;
        rightinRange = false;
    }

    // Export the scan status
    public AUTO_STATE state() {
        return state;
    }

    public boolean isDone() {
        return (state == AUTO_STATE.DONE || state == AUTO_STATE.IDLE);
    }

    enum AUTO_STATE implements OrderedEnum {
        IDLE,
        START_LEFT,
        SAMPLE_LEFT,
        PARSE_LEFT,
        START_RIGHT,
        SAMPLE_RIGHT,
        PARSE_RIGHT,
        CALCULATE,
        DONE;

        public Distance.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    enum BARCODE {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }
}