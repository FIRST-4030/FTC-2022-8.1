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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;
import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;

@Config
@Disabled
@TeleOp(name = "ServoPosTest", group = "Test")
public class ServoTester extends OpMode{
    // Hardware
    private Servo collectorArm = null;
    private Servo mid = null;
    private Servo low = null;
    private Servo tilt = null;
    // Constants used for hardware
    private static double COLLECTOR_UP = 0.37;
    private static double COLLECTOR_DOWN = 0.9;

    // Servo position test constants
    private float servoPos = 0.5f;
    private static final float INCREMENT = 0.01f;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private InputHandler in;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Initialize test servos and buttons
        try {
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            mid = hardwareMap.get(Servo.class, "Depmid");
            low = hardwareMap.get(Servo.class, "Deplow");
            tilt = hardwareMap.get(Servo.class, "Deptilt");
            Globals.opmode = this;
            in = Globals.input(this);
            in.register("+", GAMEPAD.driver1, PAD_KEY.dpad_up);
            in.register("-", GAMEPAD.driver1, PAD_KEY.dpad_down);
        } catch (Exception e) {
            telemetry.log().add("Could not find servo");
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
        // Shows number of servoPos
        telemetry.addData("Pos:", servoPos);
        // Moving the servo position and number should increase
        if (in.down("+")) {
            servoPos += INCREMENT;
            servoPos = Math.min(1.0f, servoPos);
        } else if (in.down("-")) {  // Moving the servo position and number should decrease
            servoPos -= INCREMENT;
            servoPos = Math.max(0.0f, servoPos);
        }
        // Set position of desired servo
        collectorArm.setPosition(servoPos);
    }

    @Override
    public void stop() {
    }
}