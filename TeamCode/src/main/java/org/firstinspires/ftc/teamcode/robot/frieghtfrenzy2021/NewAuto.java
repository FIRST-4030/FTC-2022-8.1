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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "OldAuto", group = "Test")
public class NewAuto extends MultiOpModeManager {
    // Hardware
    private Drive drive;
    private DuckSpin duck;
    private Distance distance;
    private Depositor depositor;
    private Capstone capstone;
    private Servo collectorArm = null;

    // Constants
    public static boolean DEBUG = false;
    public static float DRIVE_POWER = 0.5f;
    public static double COLLECTOR_UP = 0.37;
    public static double COLLECTOR_DOWN = 0.90;
    public static float OUT_DUCK = 0.0f;
    public static float OUT_WARE = 0.0f;
    public static int TURN_HUB_DUCK = 0;
    public static int TURN_HUB_WARE = 0;
    public static float HUB_DUCK = 0.0f;
    public static float HUB_WARE = 0.0f;
    public static float BACK_DUCK = 0.0f;
    public static float BACK_WARE = 0.0f;
    public static int TURN_PARK_DUCK = 0;
    public static int TURN_PARK_WARE = 32;
    public static float PARK_DUCK = -24.8f;
    public static float PARK_WARE = -40.0f;

    // Members
    private ElapsedTime runtime = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.DONE;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private InputHandler in;
    private boolean waitDepositor = false;
    private boolean  waitDrive = false;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new Depositor());
            super.register(new Capstone());
            super.register(new Drive());
            super.register(new DuckSpin());
            super.register(new Distance());
            super.register(new Collector());

            drive = new Drive();
            super.register(drive);
            duck = new DuckSpin();
            super.register(duck);
            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);
            capstone = new Capstone();
            super.register(capstone);
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");

            in = Globals.input(this);

            distance.startScan();

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
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
        if (gamepad1.dpad_right) redAlliance = true;
        if (gamepad1.dpad_left) redAlliance = false;
        if (gamepad1.dpad_up) duckSide = true;
        if (gamepad1.dpad_down) duckSide = false;
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Direction", duckSide ? "Duck" : "Warehouse");
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        driveStop();
        distance.startScan();
        state = AUTO_STATE.BARCODE;
    }

    @Override
    public void loop() {
        // Sub-modules
        //super.loop();
        depositor.loop();
        distance.loop();
        in.loop();

        //log what state it currently is in
        telemetry.addData("Auto Step: ", state);

        // Do not advance the state when we're waiting for motion
        if (waitDrive) {
            if (driveIsDone() && (!DEBUG || gamepad1.a)) {
                waitDrive = false;
                advWhenAllDone();
            }
            return;
        }
        if (waitDepositor) {
            if (depositor.isDone() && (!DEBUG || gamepad1.a)) {
                waitDepositor = false;
                advWhenAllDone();
            }
            return;
        }

        // Test turns
        if (gamepad1.b) {
            drive.turnTo(DRIVE_POWER, 90);
            state = AUTO_STATE.DONE;
            waitDrive = true;
            return;
        }

        telemetry.addData("LDrive Pos: ", drive.leftDrivePos());
        telemetry.addData("RDrive Pos: ", drive.rightDrivePos());

        // Step through the auto commands
        telemetry.log().add(state.name());
        switch (state) {
            // new routine
            case BARCODE:
                if (distance.state() == Distance.AUTO_STATE.DONE) {
                    if (distance.position() == Distance.BARCODE.LEFT) {
                        depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                    } else if (distance.position() == Distance.BARCODE.CENTER) {
                        depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
                    } else if (distance.position() == Distance.BARCODE.RIGHT) {
                        depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                    } else {
                        if (!duckSide) {
                            if (redAlliance) {
                                depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                            } else {
                                depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                            }
                        }
                    }
                    state = state.next();
                }
                break;

            case OUT_FROM_WALL:
                if (!duckSide) {
                    drive.driveTo(DRIVE_POWER, 15f);
                } else {
                    drive.driveTo(DRIVE_POWER, 24.5f);
                }
                collectorArm.setPosition(COLLECTOR_DOWN);
                depositor.prep();
                waitDepositor = true;
                waitDrive = true;
                break;

            case TURN_TO_HUB:
                if (duckSide) {
                    if (redAlliance) {
                        drive.turnTo(DRIVE_POWER, 62);
                    } else {
                        drive.turnTo(DRIVE_POWER, -62);
                    }
                } else {
                    if (redAlliance) {
                        drive.turnTo(DRIVE_POWER, -58);
                    } else {
                        drive.turnTo(DRIVE_POWER, 58);
                    }
                }
                waitDrive = true;
                break;

            case ALIGN_TO_HUB:
                if (duckSide) {
                    drive.driveTo(DRIVE_POWER, 7.2f);
                } else {
                    drive.driveTo(DRIVE_POWER, 14f);
                }
                waitDepositor = true;
                waitDrive = true;
                break;

            case DEPOSIT:
                depositor.deposit();
                waitDepositor = true;
                waitDrive = true;
                break;

            case BACK_UP:
                if (duckSide) {
                    drive.driveTo(-DRIVE_POWER, -7.5f);
                } else {
                    drive.driveTo(-DRIVE_POWER, -14f);
                }
                collectorArm.setPosition(COLLECTOR_UP);
                waitDrive = true;
                break;

            case TURN_TO_PARK:
                if (duckSide) {
                    if (redAlliance) {
                        drive.turnTo(DRIVE_POWER, 26);
                    } else {
                        drive.turnTo(DRIVE_POWER, -26);
                    }
                } else {
                    if (redAlliance) {
                        drive.turnTo(DRIVE_POWER, -TURN_PARK_WARE);
                    } else {
                        drive.turnTo(DRIVE_POWER, TURN_PARK_WARE);
                    }
                }
                waitDrive = true;
                break;

            case PARK:
                if (duckSide) {
                    drive.driveTo(-DRIVE_POWER, PARK_DUCK);
                } else {
                    drive.driveTo(-DRIVE_POWER, PARK_WARE);
                }
                waitDrive = true;
                break;
            // Stop processing
            case DONE:
                driveStop();
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        BARCODE,
        OUT_FROM_WALL,
        TURN_TO_HUB,
        ALIGN_TO_HUB,
        DEPOSIT,
        BACK_UP,
        TURN_TO_PARK,
        PARK,
        DONE;

        public NewAuto.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        drive.driveStop();
    }

    public boolean driveIsDone() {
        return (!drive.isBusy() && drive.isDone());
    }

    public void advWhenAllDone() {
        if (!waitDepositor && !waitDrive) {
            state = state.next();
        }
    }
}