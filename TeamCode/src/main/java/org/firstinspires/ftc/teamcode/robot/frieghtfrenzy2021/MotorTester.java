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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;

@Config
@Disabled
@TeleOp(name = "MotorTester", group = "Test")
public class MotorTester extends OpMode{
    // Hardware
    private DcMotor duck = null;
    private DcMotor duck2 = null;

    // Motor test constants
    public static double Power = 0;
    private static final double INCREMENT = 0.01;

    // Members
    private InputHandler in;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Collector
        try {
            duck = hardwareMap.get(DcMotor.class, "duck");
            duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            duck2 = hardwareMap.get(DcMotor.class, "duck2");
            duck2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Globals.opmode = this;
            in = Globals.input(this);
            in.register("+", GAMEPAD.driver1, PAD_KEY.dpad_up);
            in.register("-", GAMEPAD.driver1, PAD_KEY.dpad_down);
        } catch (Exception e) {
            telemetry.log().add("Could not find motor");
            error = true;
        }

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
    }

    @Override
    public void loop() {
        in.loop();
        // Shows number of Power
        telemetry.addData("Power:", Power);
        // Increasing power and number should increase
        if (in.down("+")) {
            Power += INCREMENT;
            Power = Math.min(1.0, Power);
            // Decreasing power and number should decrease
        } else if (in.down("-")) {
            Power -= INCREMENT;
            Power = Math.max(-1.0, Power);
        }
        // Set power of desired motor
        if (gamepad1.a) {
            duck.setPower(Power);
            duck2.setPower(Power);
        } else {
            duck.setPower(0);
            duck2.setPower(0);
        }
    }

    @Override
    public void stop() {
    }
}