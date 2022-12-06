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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
@Autonomous(name = "NewDriveTest", group = "Test")
public class NewDriveTest extends MultiOpModeManager {
    // Hardware
    private NewNewDrive drive;
    private Servo collectorArm = null;
    private Distance distance;
    private Depositor depositor;
    private DuckSpin duck;

    // Constants
    public static double speedMin = 0.1;
    public static double speedMax = 0.9;
    public static double COLLECTOR_UP = 0.6;
    public static double moveDistance = 30;
    public static int num = 0;
    public static int waitTime = 1;

    // Members
    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;
    private InputHandler in;
    private final ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {
        boolean error = false;
        telemetry.addData("Status", "Initializing...");

        /*try {
            super.register(new Depositor());
            super.register(new Capstone());
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

            distance.startScan();

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }*/

        try {
            super.register(new NewNewDrive());

            drive = new NewNewDrive();
            super.register(drive);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            super.init();
        } catch (Exception e) {
            telemetry.log().add(String.valueOf(e));
            error = true;
        }

        try {
            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            //collector = hardwareMap.get(DcMotor.class, "Collector");
        } catch (Exception e) {
            telemetry.log().add("Could not find collector");
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
    }

    @Override
    public void start() {
        super.start();
        num = 0;
        drive.setDoneFalse();
        state = AUTO_STATE.TEST_MOVE1;
    }

    @Override
    public void loop() {
        /*depositor.loop();
        distance.loop();
        duck.loop();
        in.loop();*/

        if (state != oldState && state != AUTO_STATE.WAIT) {
            oldState = state;
        }
        // Step through the auto commands
        switch (state) {
            case TEST_MOVE1:
                drive.arcTo(0, moveDistance, speedMin, speedMax);
                //drive.combinedCurves(0, 10, speedMin, speedMax, 0, 10, speedMin, speedMax);
                collectorArm.setPosition(COLLECTOR_UP);
                if (drive.isDone() && !drive.isBusy()) {
                    waitTimer.reset();
                    state = AUTO_STATE.WAIT;
                }
                break;
            case TEST_MOVE2:
                drive.arcTo(0, -moveDistance, -speedMin, -speedMax);
                //drive.combinedCurves(0, 10, speedMin, speedMax, 0, 10, speedMin, speedMax);
                collectorArm.setPosition(COLLECTOR_UP);
                if (drive.isDone() && !drive.isBusy()) {
                    waitTimer.reset();
                    state = AUTO_STATE.WAIT;
                }
                break;
            case WAIT:
                if (waitTimer.seconds() >= waitTime) {
                    drive.setDoneFalse();
                    if (oldState == AUTO_STATE.TEST_MOVE2) {
                        state = AUTO_STATE.TEST_MOVE1;
                    } else if (oldState == AUTO_STATE.TEST_MOVE1) {
                        state = AUTO_STATE.TEST_MOVE2;
                    }
                }
                break;
            // Stop processing
            case DONE:
                break;
        }

        //log what state it currently is in
        telemetry.addData("Auto Step: ", state);
        telemetry.addData("left ticks", drive.leftPos());
        telemetry.addData("right ticks", drive.rightPos());
        telemetry.addData("leftVel", drive.leftVel());
        telemetry.addData("rightVel", drive.rightVel());
        telemetry.update();
    }

    @Override
    public void stop() {
        state = AUTO_STATE.DONE;
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        TEST_MOVE1,
        TEST_MOVE2,
        WAIT,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}