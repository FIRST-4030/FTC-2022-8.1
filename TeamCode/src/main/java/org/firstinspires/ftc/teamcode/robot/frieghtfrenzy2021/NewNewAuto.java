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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "Autonomous", group = "Test")
public class NewNewAuto extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;
    private Distance distance;
    private Depositor depositor;
    private DuckSpin duck;
    private DcMotor collector;
    private Servo collectorArm;
    private TouchSensor sensorCollector;

    // Constants
    private final double TICKS_PER_INCH = 44.5;
    public static double speedMin = 0.1;
    public static double speedMax = 0.8;
    public static double r1 = 30.5;
    public static double arcLength1 = 25.5;
    public static double r2wh = 6;
    public static double arcLength2wh = 11.5;
    public static double r2wh2 = 6;
    public static double arcLength2wh2 = 17.25;
    public static double r2wh3 = 0;
    public static double arcLength2wh3 = 24;
    public static double r2duck = 18.5;
    public static double arcLength2duck = 41;
    public static double r3wh = 0;
    public static double arcLength3wh = 24;
    public static double r3wh2 = 14;
    public static double arcLength3wh2 = 18;
    public static double r3wh3 = 0;
    public static double arcLength3wh3 = 7.5;
    public static double r3duck = 0;
    public static double arcLength3duck = 28;
    public static double r3duck2 = 0;
    public static double arcLength3duck2 = 14.5;
    public static double r4wh = 0;
    public static double arcLength4wh = 3;
    public static double r4wh2 = 14;
    public static double arcLength4wh2 = 18;
    public static double r4wh3 = 0;
    public static double arcLength4wh3 = 28;
    public static double collectDistance = 16;
    public static double COLLECTOR_UP = 0.53;
    public static double COLLECTOR_DOWN = 0.90;
    public static int num = 0;
    public static int delayTime = 0;
    public static double DELAY_TIME = 0.75;
    public static double EJECT_TIME = 2.5;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;
    private InputHandler in;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();
    private boolean startedCollecting = false;
    private ElapsedTime collectorTimer = new ElapsedTime();
    private collectCmd collectCmdState;
    private boolean collected = false;
    private boolean started = false;
    private boolean collectStep = false;

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        // initializes opmodes of distance sensor, depositor, and duck spinner
        // uses register function from MultiOpModeManager
        try {
            super.register(new Depositor());
            super.register(new Distance());
            super.register(new DuckSpin());

            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);
            duck = new DuckSpin();
            super.register(duck);

            Globals.opmode = this;
            in = Globals.input(this);
            in.register("+", GAMEPAD.driver2, PAD_KEY.dpad_up);
            in.register("-", GAMEPAD.driver2, PAD_KEY.dpad_down);

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        // Registers and initializes drive with drive functions in NewNewDrive class
        try {
            super.register(new NewNewDrive());

            drive = new NewNewDrive();
            super.register(drive);

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        // Collector
        try {
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            collectorArm.setPosition(0.41);

            sensorCollector = hardwareMap.get(TouchSensor.class, "DC");
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }

        // Initialization status
        String status = "Ready";
        if (error) {
            status = "Hardware Error";
        }
        telemetry.addData("Status", status);
        drive.enableLogging();
    }

    @Override
    public void init_loop() {
        // button inputs changes redAlliance boolean to true or false
        // boolean determines running red or blue alliance code
        if (gamepad1.dpad_right) redAlliance = true;
        if (gamepad1.dpad_left) redAlliance = false;
        // button inputs changes duckSide boolean to true or false
        // boolean determines running duck side or warehouse side code
        if (gamepad1.dpad_up) duckSide = true;
        if (gamepad1.dpad_down) duckSide = false;
        in.loop();
        if (in.down("+")) {
            delayTime += 1;
        } else if (in.down("-") && delayTime > 0) {
            delayTime -= 1;
        }

        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Direction", duckSide ? "Duck" : "Warehouse");
        telemetry.addData("DelayTime (seconds) ", delayTime);

        // starts scanning with distance sensors during initialization
        if (distance.isDone()) {
            distance.startScan();
            if (distance.position() == Distance.BARCODE.LEFT) {
                depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
            } else if (distance.position() == Distance.BARCODE.CENTER) {
                depositor.setDoor(Depositor.DOOR_USED.MID_DOOR);
            } else if (distance.position() == Distance.BARCODE.RIGHT) {
                depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
            } else {
                // logic for guessing if element pos not sensed (NONE)
                if (!duckSide) {
                    if (redAlliance) {
                        depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                    } else {
                        depositor.setDoor(Depositor.DOOR_USED.HIGH_DOOR);
                    }
                }
            }
        }
        telemetry.addData("Barcode Pos: ", distance.position());
        telemetry.addData("Door: ", depositor.doorUsed());
        // calls and runs all other init_loop() functions from registered opmodes
        super.init_loop();
    }

    @Override
    public void start() {
        // calls and runs all other start() functions from registered opmodes
        super.start();
        // resetting variables at the start
        num = 0;
        collectDistance = 9;
        collectStep = false;
        drive.setDoneFalse();
        state = AUTO_STATE.ARC;
        delayTimer.reset();
        runTime.reset();
    }

    @Override
    public void loop() {
        // conditiion to check delayTime,
        // when the set delayTime is reached, start everything
        if (delayTimer.seconds() >= delayTime) {
            // calls and runs all other loop() functions from registered opmodes
            depositor.loop();
            distance.loop();
            duck.loop();
            if (state != oldState) {
                oldState = state;
            }
            // duck side autonomous steps
            if (duckSide) {
                switch (state) {
                    case ARC:
                        depositor.prep();
                        if (redAlliance) {
                            drive.arcToNew(-r1, arcLength1, speedMin, speedMax);
                        } else {
                            drive.arcToNew(r1, arcLength1, speedMin, speedMax);
                        }
                        collectorArm.setPosition(COLLECTOR_UP);
                        collectCmdState = collectCmd.IDLE;
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.PREP_WAIT;
                        }
                        break;
                    case PREP_WAIT:
                        if (depositor.isDone()) {
                            depositor.deposit();
                            state = AUTO_STATE.DEPOSIT;
                        }
                        break;
                    case DEPOSIT:
                        if (depositor.isDone()) {
                            state = AUTO_STATE.PARK;
                        }
                        break;
                    case PARK:
                        if (redAlliance) {
                            drive.arcToNew(r2duck, -arcLength2duck, -speedMin, -speedMax);
                        } else {
                            drive.arcToNew(-r2duck, -arcLength2duck, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD1;
                        }
                        break;
                    case ADD1:
                        if (redAlliance) {
                            drive.arcToNew(r3duck, arcLength3duck, speedMin, speedMax);
                        } else {
                            drive.arcToNew(r3duck, arcLength3duck - 2, speedMin, speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.DUCK_SPIN;
                        }
                        break;
                    case DUCK_SPIN:
                        duck.auto(redAlliance);
                        if (duck.isDone()) {
                            state = AUTO_STATE.ADD12;
                        }
                        break;
                    case ADD12:
                        if (redAlliance) {
                            drive.arcToNew(r3duck2, -arcLength3duck2, -speedMin, -speedMax);
                        } else {
                            drive.arcToNew(-r3duck2, -arcLength3duck2 + .25, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD22;
                        }
                        break;
                    case ADD22:
                        /* if (!drive.isDone()) {
                            if (redAlliance) {
                                drive.arcTo(-r4duck, -arcLength4duck, -speedMin, -speedMax);
                            } else {
                                drive.arcTo(r4duck, -arcLength4duck, -speedMin, -speedMax);
                            }
                        }
                        COLLECTOR_UP = 0.41;
                        collectorArm.setPosition(COLLECTOR_UP);
                        if (drive.isDone() && !drive.isBusy()) {*/
                        depositor.reset();
                        drive.setDoneFalse();
                        state = AUTO_STATE.LAST;
                        //}
                        break;
                    case LAST:
                        if (depositor.isDone()) {
                            depositor.tiltBack();
                            drive.setDoneFalse();
                            state = AUTO_STATE.DONE;
                        }
                        break;
                    // Stop processing
                    case DONE:
                        break;
                }
            } else {    // Warehouse Side Steps
                switch (state) {
                    case ARC:
                        depositor.prep();
                        if (redAlliance) {
                            drive.arcTo(r1, arcLength1, speedMin, speedMax);
                        } else {
                            drive.arcTo(-r1, arcLength1, speedMin, speedMax);
                        }
                        collectorArm.setPosition(COLLECTOR_UP);
                        collectCmdState = collectCmd.IDLE;
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.PREP_WAIT;
                        }
                        break;
                    case PREP_WAIT:
                        if (depositor.isDone()) {
                            depositor.deposit();
                            state = AUTO_STATE.DEPOSIT;
                        }
                        break;
                    case DEPOSIT:
                        if (depositor.isDone()) {
                            state = AUTO_STATE.PARK;
                        }
                        break;
                    case PARK:
                        if (redAlliance) {
                            drive.arcTo(r2wh, -arcLength2wh, -speedMin, -speedMax);
                        } else {
                            drive.arcTo(-r2wh, -arcLength2wh, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.PARK2;
                        }
                        break;
                    case PARK2:
                        if (redAlliance) {
                            drive.arcTo(-r2wh2, -arcLength2wh2, -speedMin, -speedMax);
                        } else {
                            drive.arcTo(r2wh2, -arcLength2wh2, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.PARK3;
                        }
                        break;
                    case PARK3:
                        if (redAlliance) {
                            drive.arcTo(r2wh3, -arcLength2wh3, -speedMin, -speedMax);
                        } else {
                            drive.arcTo(-r2wh3, -arcLength2wh3, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            depositor.reset();
                            drive.setDoneFalse();
                            state = AUTO_STATE.COLLECT2;
                        }
                        break;
                    case COLLECT:
                        if (!startedCollecting) {
                            collectCmdState = collectCmd.BEFORE_COLLECT;
                            startedCollecting = true;
                        }
                        if (!drive.isDone()) {
                            drive.arcTo(0, -collectDistance, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy() && collectCmdState == collectCmd.EJECT) {
                            drive.setDoneFalse();
                            startedCollecting = false;
                            state = AUTO_STATE.RETURN;
                        } else if (collectCmdState == collectCmd.COLLECT && collectorTimer.seconds() > 6) {
                            // if collector is collecting for more than 6 seconds
                            // and no freight has triggered the sensor
                            // stop the whole process, run the last step and park
                            collectCmdState = collectCmd.IDLE;
                            depositor.reset();
                            state = AUTO_STATE.LAST;
                        }
                        break;
                    case RETURN:
                        drive.setDoneFalse();
                        state = AUTO_STATE.ADD1;
                        break;
                    case ADD1:
                        depositor.prep();
                        if (!collectStep) {
                            if (redAlliance) {
                                drive.arcTo(-r3wh, collectDistance + arcLength3wh, speedMin, speedMax);
                            } else {
                                drive.arcTo(r3wh, collectDistance + arcLength3wh, speedMin, speedMax);
                            }
                        } else {
                            if (redAlliance) {
                                drive.arcTo(-r3wh, collectDistance + arcLength3wh + 3, speedMin, speedMax);
                            } else {
                                drive.arcTo(r3wh, collectDistance + arcLength3wh + 3, speedMin, speedMax);
                            }
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD12;
                        }
                        break;
                    case ADD12:
                        if (redAlliance) {
                            drive.arcTo(-r3wh2, arcLength3wh2, speedMin, speedMax);
                        } else {
                            drive.arcTo(r3wh2, arcLength3wh2, speedMin, speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD13;
                        }
                        break;
                    case ADD13:
                        if (redAlliance) {
                            drive.arcTo(r3wh3, arcLength3wh3, speedMin, speedMax);
                        } else {
                            drive.arcTo(r3wh3, arcLength3wh3, speedMin, speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy() && depositor.isDone()) {
                            depositor.deposit();
                            drive.setDoneFalse();
                            state = AUTO_STATE.DUCK_SPIN;
                        }
                        break;
                    case DUCK_SPIN:
                        if (depositor.isDone()) {
                            state = AUTO_STATE.ADD2;
                        }
                        break;
                    case ADD2:
                        if (!drive.isDone()) {
                            if (redAlliance) {
                                drive.arcTo(-r4wh, -arcLength4wh, -speedMin, -0.6);
                            } else {
                                drive.arcTo(r4wh, -arcLength4wh, -speedMin, -0.6);
                            }
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            depositor.reset();
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD22;
                        }
                        break;
                    case ADD22:
                        if (!drive.isDone()) {
                            if (redAlliance) {
                                drive.arcTo(-r4wh2, -arcLength4wh2, -speedMin, -speedMax);
                            } else {
                                drive.arcTo(r4wh2, -arcLength4wh2, -speedMin, -speedMax);
                            }
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            state = AUTO_STATE.ADD23;
                        }
                        break;
                    case ADD23:
                        if (!drive.isDone()) {
                            if (redAlliance) {
                                drive.arcTo(-r4wh3, -arcLength4wh3, -speedMin, -speedMax);
                            } else {
                                drive.arcTo(r4wh3, -arcLength4wh3, -speedMin, -speedMax);
                            }
                        }
                        if (drive.isDone() && !drive.isBusy()) {
                            drive.setDoneFalse();
                            //collectDistance += 2;
                            state = AUTO_STATE.COLLECT2;
                        }
                        break;
                    case COLLECT2:
                        /* if (!startedCollecting) {
                            collectCmdState = collectCmd.BEFORE_COLLECT;
                            startedCollecting = true;
                        } */
                        if (!drive.isDone()) {
                            drive.arcTo(0, -collectDistance, -speedMin, -speedMax);
                        }
                        if (drive.isDone() && !drive.isBusy()) { // && collectCmdState == collectCmd.EJECT) {
                            drive.setDoneFalse();
                            startedCollecting = false;
                            state = AUTO_STATE.LAST;
                        }
                        break;
                    case LAST:
                        if (depositor.isDone() && collectCmdState == collectCmd.IDLE) {
                            depositor.tiltBack();
                            drive.setDoneFalse();
                            state = AUTO_STATE.DONE;
                        }
                        break;
                    // Stop processing
                    case DONE:
                        break;
                }
            }

            telemetry.addData("collector pos: ", collectorArm.getPosition());
            telemetry.addData("collector state: ", collectCmdState);

            // Collector state
            collected = sensorCollector.isPressed();
            telemetry.addData("Collected? ", collected);
            switch (collectCmdState) {
                case IDLE:
                    break;
                case BEFORE_COLLECT:
                    collectorTimer.reset();
                    collectCmdState = collectCmd.COLLECT;
                    break;
                case COLLECT:
                    if (collected && collectorTimer.seconds() > (Math.PI / 10)) {
                        collectorTimer.reset();
                        collectCmdState = collectCmd.SENSOR_DELAY;
                    }
                    break;
                case SENSOR_DELAY:
                    collected = false;
                    if (collectorTimer.seconds() > DELAY_TIME) {
                        collectCmdState = collectCmd.BEFORE_EJECT;
                    }
                    break;
                case BEFORE_EJECT:
                    collectorTimer.reset();
                    collectCmdState = collectCmd.EJECT;
                    break;
                case EJECT:
                    if (collectorTimer.seconds() > EJECT_TIME) {
                        collectCmdState = collectCmd.IDLE;
                    }
                    break;
            }

            // Collector commands
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

            //log what state it currently is in
            telemetry.addData("Auto Step: ", state);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        ARC,
        PREP_WAIT,
        DEPOSIT,
        PARK,
        PARK2,
        PARK3,
        COLLECT,
        RETURN,
        ADD1,
        ADD12,
        ADD13,
        DUCK_SPIN,
        ADD2,
        ADD22,
        ADD23,
        COLLECT2,
        LAST,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    enum collectCmd {
        IDLE,
        BEFORE_COLLECT,
        COLLECT,
        SENSOR_DELAY,
        BEFORE_EJECT,
        EJECT
    }
}
