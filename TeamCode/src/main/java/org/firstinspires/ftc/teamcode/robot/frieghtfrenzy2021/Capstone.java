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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
//@TeleOp(name = "Capstone", group = "Test")
public class Capstone extends OpMode {
    // Hardware
    private Servo arm = null;

    // Config
    public static boolean DEBUG = false;
    public static double CAP_IN = 0;
    public static double CAP_MID = 0.5;
    public static double CAP_DOWN = 0.87;
    public static double CAPSTONE_DELTA = 0.01;

    // Members
    private boolean enabled = false;
    private double delay = 0.1;
    private double target = 0;
    private AUTO_STATE state = AUTO_STATE.DONE;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Capstone
        try {
            arm = hardwareMap.get(Servo.class, "Caparm");
            arm.setPosition(CAP_IN);

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
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        arm.setPosition(CAP_MID);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        switch(state) {
            case ARM_DOWN:
                target = CAP_DOWN;
                arm.setPosition(CAP_DOWN);
                if (arm.getPosition() == target) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case ARM_UP:
                target = CAP_MID;
                arm.setPosition(CAP_MID);
                if (arm.getPosition() == target) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case ARM_IN:
                target = CAP_IN;
                arm.setPosition(CAP_IN);
                if (arm.getPosition() == target) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case MANUAL:
                if (target >= 0 && target <= 1) {
                    target += (gamepad2.right_stick_y * 0.02);
                }
                break;
            case DONE:
                break;
        }

        // Capstone
        if (gamepad2.dpad_down) {
            state = AUTO_STATE.ARM_DOWN;
        } else if (gamepad2.dpad_up) {
            state = AUTO_STATE.ARM_UP;
        }


        /* double capError = target - arm.getPosition();
        if (capError != 0 && timer.seconds() > delay) {
            double delta = Math.max(CAPSTONE_DELTA, Math.abs(capError));
            delta *= Math.signum(capError);
            arm.setPosition(arm.getPosition() + delta);
            timer.reset();
        } */

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Capstone Output", "%.2f",
                    arm.getPosition());
        }
    }

    public void armUp() {
        state = AUTO_STATE.ARM_UP;
    }

    public void armDown() {
        state = AUTO_STATE.ARM_DOWN;
    }

    public boolean isDone() {
        return state == AUTO_STATE.DONE;
    }

    @Override
    public void stop() {
    }

    enum AUTO_STATE implements OrderedEnum {
        ARM_DOWN,
        ARM_UP,
        ARM_IN,
        MANUAL,
        DONE;

        public Capstone.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}