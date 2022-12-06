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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Disabled
@TeleOp(name = "TimerTest", group = "Test")
public class TimerTest extends OpMode {

    private static double DUCK_POWER = 0;
    private static double duckPowerMin = 0.65;  // min duck spinner speed (0 - 1.0)
    private static double duckPowerMax = 0.9;  // max duck spinner speed (0 - 1.0)
    private static double duckRampTime = 1.25;  // duck spinner ramp time (seconds, >0)
    private static double timerRatio = 0;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime startRampTime1 = new ElapsedTime();

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Duck spinner
        if (gamepad1.a) startRampTime1.reset();
        timerRatio = Math.max(Math.min(startRampTime1.seconds() / duckRampTime, 1.0), 0);
        if (timerRatio != 0.0 && timerRatio != 1.0) {
            DUCK_POWER = duckPowerMin + timerRatio * (duckPowerMax - duckPowerMin);
        } else {
            DUCK_POWER = 0.0;
        }
        telemetry.addData("Timer Ratio:", timerRatio);
        telemetry.addData("Runtime:", runtime.seconds());
        telemetry.addData("Runtime:", startRampTime1.seconds());
        telemetry.addData("Duck Power:", DUCK_POWER);
    }

    @Override
    public void stop() {
    }
}