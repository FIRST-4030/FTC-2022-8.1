package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;

@Disabled
@TeleOp(name = "MOMM_Teleop", group = "MOMM")
public class MOMM_Teleop extends MultiOpModeManager {
    private Drive drive;
    private DuckSpin duck;
    private Distance distance;
    private Depositor depositor;
    private Collector collector;
    private Capstone capstone;

    public static int TURN_SMALL = 5;
    public static float TURN_SPEED = 0.4f;

    @Override
    public void init() {
        boolean error = false;
        try {
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
            collector = new Collector();
            super.register(collector);

            input.register("BARCODE", GAMEPAD.driver1, PAD_KEY.guide);
            input.register("DUCK_RED", GAMEPAD.driver1, PAD_KEY.b);
            input.register("DUCK_BLUE", GAMEPAD.driver1, PAD_KEY.a);
            /* input.register("TURN_CW", GAMEPAD.driver1, PAD_KEY.a);
            input.register("TURN_CCW", GAMEPAD.driver1, PAD_KEY.b); */

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
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        input.loop();

        // Start the auto method for the duck spinner
        if (duck.isDone()) {
            if (input.down("DUCK_RED")) {
                duck.teleop(true);
            } else if (input.down("DUCK_BLUE")) {
                duck.teleop(false);
            }
        }

        /*// Small angle turns
        if (input.down("TURN_CW")) {
            drive.turnTo(TURN_SPEED, TURN_SMALL);
        }
        if (input.down("TURN_CCW")) {
            drive.turnTo(-TURN_SPEED, -TURN_SMALL);
        } */

        // Trigger a distance scan manually at any time
        if (input.down("BARCODE")) {
            distance.startScan();
        }
        // Trigger updates every 10 seconds, but not until the scan has run at least once
        if (distance.age() > 10 && distance.state() != Distance.AUTO_STATE.IDLE) {
            distance.startScan();
        }
        // Distance scan status and result
        telemetry.addData("Barcode", "P %s, A %.2f, S %s",
                distance.position(), distance.age(), distance.state());
    }

    @Override
    public void stop() {
        super.stop();
    }
}