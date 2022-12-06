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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;

@Config
@Disabled
@TeleOp(name = "NewTeleOp", group = "Test")
// inherits MultiOpModeManager which allows calling functions and methods from multiple different OpMode classes
public class NewTeleOp extends MultiOpModeManager {
    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor duckSpinner = null;
    private DcMotor duckSpinner2 = null;
    private DcMotor collector = null;
    private Servo collectorArm = null;
    private Servo capstoneArm = null;
    private DistanceSensor distanceLeft = null;
    private DistanceSensor distanceRight = null;
    private TouchSensor sensorCollector = null;
    private Depositor depositor;
    private DuckSpin duckSpin;

    // Constants used for hardware
    private static double DUCK_POWER = 0.0;
    public static double COLLECTOR_UP = 0.53;
    public static double COLLECTOR_DOWN = 0.90;
    private static double SPEED = 1;
    public static int DISTANCE = 30;
    public static double DELAY_TIME = 1.75;
    public static double EJECT_TIME = 2;
    private static double timerRatio = 0.0;
    public static double duckPowerMin = 0.63;  // min duck spinner speed (0 - 1.0)
    public static double duckPowerMax = 0.88;  // max duck spinner speed (0 - 1.0)
    public static double duckRampTime = 1.4;  // duck spinner ramp time (seconds, >0)
    public static double CAP_IN = 0;
    //private static double CAP_UP = 0.35;
    public static double CAP_MID = 0.5;
    public static double CAP_DOWN = 0.87;
    private static double CAPSTONE_DELTA = 0.01;
    private static double delayTime = 0.1;
    private static double capstoneTarget = 0;
    private double LEFT_DRIVE_POW = 0;
    private double RIGHT_DRIVE_POW = 0;
    private static double ACCEL_CONSTANT = 0.4;
    private static double fastFactor = 0;
    private static double fastFactor2 = 0;
    collectCmd collectCmdState = collectCmd.IDLE;
    private static double collectStartDelay = (Math.PI / 10);

    private static boolean collectorActive = false;
    private static boolean collected = false;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime duckTimer = new ElapsedTime();
    private ElapsedTime capTimer = new ElapsedTime();
    private ElapsedTime collectorTimer = new ElapsedTime(0);

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // Drive Motors
        try {
            leftDrive = hardwareMap.get(DcMotor.class, "BL");
            rightDrive = hardwareMap.get(DcMotor.class, "BR");
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.log().add("Could not find drive");
            error = true;
        }

        // Duck Spinner
        try {
            /*duckSpinner = hardwareMap.get(DcMotor.class, "duck");
            duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            duckSpinner2 = hardwareMap.get(DcMotor.class, "duck2");
            duckSpinner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
            super.register(new DuckSpin());
            duckSpin = new DuckSpin();
            super.register(duckSpin);
            duckSpin.init();
        } catch (Exception e) {
            telemetry.log().add("Could not find duck spinner");
            error = true;
        }

        // Depositor
        try {
            super.register(new Depositor());
            depositor = new Depositor();
            super.register(depositor);
            depositor.init();
        } catch (Exception e) {
            telemetry.log().add("Could not find depositor");
            error = true;
        }


        // Collector
        try {
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            sensorCollector = hardwareMap.get(TouchSensor.class, "DC");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
            error = true;
        }

        // Capstone Grabber
        try {
            capstoneArm = hardwareMap.get(Servo.class, "Caparm");
        } catch (Exception e) {
            telemetry.log().add("Could not find capstone dep");
            error = true;
        }

        // Distance Sensors
        try {
            distanceLeft = hardwareMap.get(DistanceSensor.class, "DL");
            distanceRight = hardwareMap.get(DistanceSensor.class, "DR");
        } catch (Exception e) {
            telemetry.log().add("Could not find range sensors");
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
        runtime.reset();

        // Zero the drive encoders and enable RUN_USING_ENCODER for velocity PID
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorArm.setPosition(COLLECTOR_UP);
        capstoneArm.setPosition(CAP_MID);
        super.start();
    }

    @Override
    public void loop() {
        // Tank drive
        // turbo/acceleration system
        // left trigger for acceleration of left drive when it is pushed down enough
        // (more than default speed proportion "ACCEL_CONSTANT")
        if (gamepad1.left_trigger > ACCEL_CONSTANT) {
            fastFactor = gamepad1.left_trigger;
        } else {
            fastFactor = ACCEL_CONSTANT;
        }
        // right trigger for acceleration of right drive when it is pushed down enough
        // (more than default speed proportion "ACCEL_CONSTANT")
        if (gamepad1.right_trigger > ACCEL_CONSTANT) {
            fastFactor2 = gamepad1.right_trigger;
        } else {
            fastFactor2 = ACCEL_CONSTANT;
        }
        // left joystick for left drive
        LEFT_DRIVE_POW = Math.pow(-gamepad1.left_stick_y, 1);
        // right joystick for right drive
        RIGHT_DRIVE_POW = Math.pow(-gamepad1.right_stick_y, 1);
        leftDrive.setPower(LEFT_DRIVE_POW * fastFactor);
        rightDrive.setPower(RIGHT_DRIVE_POW * fastFactor2);

        // Duck spinner
        duckSpin.loop();

        // Depositor
        depositor.loop();
        // When depositor is preparing freight and collector is idle,
        // change its state to ejecting to prevent any stuckness
        if (!depositor.isDone() && depositor.state == Depositor.AUTO_STATE.DOOR_PREP && collectCmdState == collectCmd.IDLE) {
            collectCmdState = collectCmd.BEFORE_EJECT;
        }

        // Collector
        // boolean variable 'collected' is only updated when it is false,
        // and when it is true, it doesn't change until the collector is no longer in the COLLECT state,
        // or if the timer is less than the collection start delay time, which means it might be a false trigger
        if (!collected) {
            collected = sensorCollector.isPressed();
        } else if (collectCmdState != collectCmd.COLLECT || collectorTimer.seconds() <= collectStartDelay) {
            collected = sensorCollector.isPressed();
        }
        telemetry.addData("Collected? ", collected);
        // Collector state
        // This current collector works by holding down a button (gamepad2 left bumper) to start collecting
        // and it goes on to ejecting (opposite direction) when button is released or the sensor is triggered
        switch (collectCmdState) {
            case IDLE:
                // IDLE state
                // Checks user input and starts collection by moving on to the BEFORE_COLLECT state,
                // if left bumper on gamepad2 is pressed
                if (gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_COLLECT;
                }
                break;
            case BEFORE_COLLECT:
                // Resets timer and move onto the COLLECT state
                collectorTimer.reset();
                collectCmdState = collectCmd.COLLECT;
                break;
            case COLLECT:
                // Checks driver input
                // If the bumper that was held to start collecting is released,
                // move on to the BEFORE_EJECT state for ejection
                if (!gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                // Checks sensor and time passed after timer reset in the BEFORE_COLLECT state
                // If the sensor is triggered, 'collected' would return true
                // And if the time in second is more than the desired start time of collection (0.314...right now),
                // timer is reset and state is changed to SENSOR_DELAY
                if (collected && collectorTimer.seconds() > collectStartDelay) {
                    collectorTimer.reset();
                    collectCmdState = collectCmd.SENSOR_DELAY;
                }
                break;
            case SENSOR_DELAY:
                collected = false;
                // this state is only for when the sensor is triggered
                // it adds a delay time before ejecting so that
                if (collectorTimer.seconds() > DELAY_TIME) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                break;
            case BEFORE_EJECT:
                // resets timer again and change state to EJECT
                collectorTimer.reset();
                collectCmdState = collectCmd.EJECT;
                break;
            case EJECT:
                // checks timer
                // When timer time is longer (greater) than the EJECT_TIME (2 seconds),
                // or when user input of gamepad2 right bumper is detected
                // stop ejection and change state to IDLE
                if (collectorTimer.seconds() > EJECT_TIME) {
                    collectCmdState = collectCmd.IDLE;
                }
                if (gamepad2.right_bumper) {
                    collectCmdState = collectCmd.IDLE;
                }
                break;
        }

        // Collector commands
        // additional commands for collectorArm servo and collector motor
        switch (collectCmdState) {
            case IDLE:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(0);
                break;
            case BEFORE_COLLECT:
            case COLLECT:
            case SENSOR_DELAY:
                collectorArm.setPosition(COLLECTOR_DOWN);
                collector.setPower(1);
                break;
            case BEFORE_EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(1);
                break;
            case EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(-1);
                break;
        }
    }

    @Override
    public void stop() {
    }

    // Collector states enum list
    enum collectCmd {
        IDLE,
        BEFORE_COLLECT,
        COLLECT,
        SENSOR_DELAY,
        BEFORE_EJECT,
        EJECT
    }
}