package org.firstinspires.ftc.teamcode.utils.momm.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;

// Extend OpMode as usual
// If you must extend MultiOpModeManger be sure to @override all of the standard methods (or else)
//
// Register with @TeleOp or @Autonomous if you want to expose this OpMode independently
// It will still work as a sub-mode in MOMM even if it is not registered for independent use
//@TeleOp(name = "MOMM_Drive", group = "Test")
@Config
public class MOMM_Drive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double ACCEL_CONSTANT = 0.4;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private InputHandler in;

    // Standard methods
    @Override
    public void init() {
        // Pull in Globals
        in = Globals.input(this);

        // Drive wheels
        try {
            // TODO: Map to actual hardware
            driveLeft = hardwareMap.get(driveLeft.getClass(), "DL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // TODO: Map to actual hardware
            driveRight = hardwareMap.get(driveRight.getClass(), "DR");
            driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            in.register("DRIVE_LEFT", GAMEPAD.driver1, PAD_KEY.left_stick_y);
            in.register("DRIVE_RIGHT", GAMEPAD.driver1, PAD_KEY.right_stick_y);
            in.register("DRIVE_ACCEL", GAMEPAD.driver1, PAD_KEY.right_trigger);

            // Don't enable this OM unless we find the necessary hardware
            // This avoids null-pointer exceptions and allows other code
            // to run normally even while this OM fails
            //
            // Be sure to protect methods that use drive hardware by checking this flag
            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        // Input
        in.loop();

        // Tank drive
        double fastFactor = ACCEL_CONSTANT;
        if (in.value("DRIVE_ACCEL") > ACCEL_CONSTANT) {
            fastFactor = in.value("DRIVE_ACCEL");
        }
        double LEFT_DRIVE_POW = Math.pow(-in.value("DRIVE_LEFT"), INPUT_SCALING_EXPONENT);
        double RIGHT_DRIVE_POW = Math.pow(-in.value("DRIVE_RIGHT"), INPUT_SCALING_EXPONENT);
        driveLeft.setPower(LEFT_DRIVE_POW * fastFactor);
        driveRight.setPower(RIGHT_DRIVE_POW * fastFactor);

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Drive Input", "L %.2f, R %.2f, A %.2f",
                    in.value("DRIVE_LEFT"), in.value("DRIVE_RIGHT"),
                    in.value("DRIVE_ACCEL"));
            telemetry.addData("Drive Output", "L %.2f/%d, R %.2f/%d",
                    driveLeft.getPower(), driveLeft.getCurrentPosition(),
                    driveRight.getPower(), driveRight.getCurrentPosition());
        }
    }

    @Override
    public void stop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Stop the drive motors
        driveLeft.setPower(0);
        driveRight.setPower(0);
    }
}
