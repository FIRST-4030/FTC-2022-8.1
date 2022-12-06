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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "secondVersionAuto", group = "Test")
public class secondVersionAuto extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;
    private Distance distance;
    private Depositor depositor;
    private Capstone capstone;
    private DuckSpin duck;
    private Collector collector;

    // Constants
    public static double speedMin = 0.1;
    public static double speedMax = 0.7;
    public static double r1 = 36.8;
    public static double arcLength1 = 26.5;
    public static double r2wh = 18;
    public static double arcLength2wh = 4;
    public static double r2wh2 = 0;
    public static double arcLength2wh2 = 40;
    public static double r2duck = 19;
    public static double arcLength2duck = 44;
    public static double r3wh = 0;
    public static double arcLength3wh = 40;
    public static double r3wh2 = 18;
    public static double arcLength3wh2 = 4;
    public static double r3duck = 0;
    public static double arcLength3duck = 42;
    public static double r4wh = 18;
    public static double arcLength4wh = 4;
    public static double r4wh2 = 0;
    public static double arcLength4wh2 = 40;
    public static double r4duck = 0;
    public static double arcLength4duck = 18;
    public static double arcLength5duck = Math.PI;
    public static double COLLECTOR_UP = 0.6;
    public static double COLLECTOR_DOWN = 0.90;
    public static int num = 0;
    public static int delayTime = 0;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private InputHandler in;
    private boolean redAlliance = false;
    private boolean duckSide = false;
    private ElapsedTime delayTimer = new ElapsedTime();

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        try {
            super.register(new Depositor());
            super.register(new Capstone());
            super.register(new Distance());
            super.register(new DuckSpin());
            super.register(new Collector());

            distance = new Distance();
            super.register(distance);
            depositor = new Depositor();
            super.register(depositor);
            capstone = new Capstone();
            super.register(capstone);
            duck = new DuckSpin();
            super.register(duck);
            collector = new Collector();
            super.register(collector);

            Globals.opmode = this;
            in = Globals.input(this);
            in.register("+", GAMEPAD.driver2, PAD_KEY.dpad_up);
            in.register("-", GAMEPAD.driver2, PAD_KEY.dpad_down);

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        try {
            super.register(new NewNewDrive());

            drive = new NewNewDrive();
            super.register(drive);

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
        drive.enableLogging();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_right) redAlliance = true;
        if (gamepad1.dpad_left) redAlliance = false;
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

        if (distance.isDone()) {
            distance.startScan();
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
                } else {
                    depositor.setDoor(Depositor.DOOR_USED.LOW_DOOR);
                }
            }
        }
        telemetry.addData("Barcode Pos: ", distance.position());
        telemetry.addData("Door: ", depositor.doorUsed());
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        num = 0;
        drive.setDoneFalse();
        state = AUTO_STATE.ARC;
        delayTimer.reset();
    }

    @Override
    public void loop() {
        if (delayTimer.seconds() >= delayTime) {
            depositor.loop();
            distance.loop();
            duck.loop();

            // Step through the auto commands
            switch (state) {
                /* case MOVE_OUT:
                    depositor.prep();
                    //drive.driveTo(speedMin, speedMax, distance1);
                    drive.arcTo(0, distance1, speedMin, speedMax);
                    collectorArm.setPosition(COLLECTOR_UP);
                    if (drive.isDone() && !drive.isBusy()) {
                        drive.setDoneFalse();
                        state = state.next();
                    }
                    break;*/
                case ARC:
                    depositor.prep();
                    if (duckSide) {
                        if (redAlliance) {
                            drive.arcTo(-r1, arcLength1, speedMin, speedMax);
                        } else {
                            drive.arcTo(r1, arcLength1, speedMin, speedMax);
                        }
                    } else {
                        if (redAlliance) {
                            drive.arcTo(r1, arcLength1, speedMin, speedMax);
                        } else {
                            drive.arcTo(-r1, arcLength1, speedMin, speedMax);
                        }
                    }
                    collector.collectorUp();
                    if (drive.isDone() && !drive.isBusy()) {
                        drive.setDoneFalse();
                        state = state.next();
                    }
                    break;
                case PREP_WAIT:
                    if (depositor.isDone()) {
                        depositor.deposit();
                        state = state.next();
                    }
                    break;
                case DEPOSIT:
                    if (depositor.isDone()) {
                        state = state.next();
                    }
                    break;
                case PARK:
                    if (duckSide) {
                        if (redAlliance) {
                            drive.arcTo(r2duck, -arcLength2duck, -speedMin, -speedMax);
                        } else {
                            drive.arcTo(-r2duck, -arcLength2duck, -speedMin, -speedMax);
                        }
                    } else {
                        if (redAlliance) {
                            //drive.arcTo(-r3wh, -arcLength3wh, -speedMin, -speedMax);
                            drive.combinedCurves(-r2wh, -arcLength2wh, -r2wh2, -arcLength2wh2, -speedMin, -speedMax);
                        } else {
                            //drive.arcTo(r3wh, -arcLength3wh, -speedMin, -speedMax);
                            drive.combinedCurves(r2wh, -arcLength2wh, r2wh2, -arcLength2wh2, -speedMin, -speedMax);
                        }
                    }
                    if (num == 0) {
                        depositor.reset();
                        num++;
                    }
                    if (drive.isDone() && !drive.isBusy()) {
                        drive.setDoneFalse();
                        state = state.next();
                    }
                    break;
                case COLLECT:
                    /* collector.autoCollect();
                    drive.slowReverse();
                    if (collector.isEjecting()) {
                        drive.stopDrive();
                        drive.returnToPos(drive.returnOffSet());*/
                        state = state.next();
                    //}
                    break;
                case ADD1:
                    if (duckSide) {
                        if (redAlliance) {
                            drive.arcTo(r3duck, arcLength3duck, speedMin, speedMax);
                        } else {
                            drive.arcTo(r3duck, arcLength3duck, speedMin, speedMax);
                        }
                    } else {
                        if (redAlliance) {
                            //drive.arcTo(-r3wh, arcLength3wh, speedMin, speedMax);
                            drive.combinedCurves(-r3wh, arcLength3wh, -r3wh2, arcLength3wh2, speedMin, speedMax);
                        } else {
                            //drive.arcTo(r3wh, arcLength3wh, speedMin, speedMax);
                            drive.combinedCurves(r3wh, arcLength3wh, r3wh2, arcLength3wh2, speedMin, speedMax);
                        }
                    }
                    if (drive.isDone() && !drive.isBusy()) {
                        drive.setDoneFalse();
                        state = state.next();
                    }
                    break;
                case DUCK_SPIN:
                    if (duckSide) {
                        duck.auto(redAlliance);
                        if (duck.isDone()) {
                            state = state.next();
                        }
                    } else {
                        state = state.next();
                    }
                    break;
                case ADD2:
                    if (duckSide) {
                        if (redAlliance) {
                            drive.arcTo(-r4duck, -arcLength4duck, -speedMin, -speedMax);
                        } else {
                            drive.arcTo(r4duck, -arcLength4duck, -speedMin, -speedMax);
                        }
                    } else {
                        if (redAlliance) {
                            //drive.arcTo(-r3wh, -arcLength3wh, -speedMin, -speedMax);
                            drive.combinedCurves(-r4wh, -arcLength4wh, -r4wh2, -arcLength4wh2, -speedMin, -speedMax);
                        } else {
                            //drive.arcTo(r3wh, -arcLength3wh, -speedMin, -speedMax);
                            drive.combinedCurves(r4wh, -arcLength4wh, r4wh2, -arcLength4wh2, -speedMin, -speedMax);
                        }
                    }
                    if (drive.isDone() && !drive.isBusy() && depositor.isDone()) {
                        drive.setDoneFalse();
                        state = state.next();
                    }
                    break;
                case LAST:
                    /* if (duckSide) {
                        drive.arcTo(0, -arcLength5duck, -speedMin, -speedMax);
                    } else {
                        depositor.tiltBack();
                        state = state.next();
                    }
                    if (drive.isDone() && !drive.isBusy()) {*/
                        depositor.tiltBack();
                        drive.setDoneFalse();
                        state = AUTO_STATE.DONE;
                    //}
                    break;
                // Stop processing
                case DONE:
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
        COLLECT,
        ADD1,
        DUCK_SPIN,
        ADD2,
        LAST,
        DONE;

        public secondVersionAuto.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}