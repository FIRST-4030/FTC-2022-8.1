package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;

@Config
//@TeleOp(name = "Drive", group = "Test")
public class Drive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
    public static double TURN_RATIO = 6.3;
    public static double ACCEL_CONSTANT = 0.4;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private boolean auto = false;
    private InputHandler in;

    // Standard methods
    @Override
    public void init() {
        // Pull in Globals
        in = Globals.input(this);

        // Drive wheels
        try {
            driveLeft = hardwareMap.get(DcMotor.class, "BL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveRight = hardwareMap.get(DcMotor.class, "BR");
            driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            in.register("DRIVE_LEFT", GAMEPAD.driver1, PAD_KEY.left_stick_y);
            in.register("DRIVE_RIGHT", GAMEPAD.driver1, PAD_KEY.right_stick_y);
            in.register("DRIVE_ACCEL", GAMEPAD.driver1, PAD_KEY.right_trigger);

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        // Input
        in.loop();

        // Skip driver control while there's an active auto command
        if (auto) {
            if (!isBusy()) {
                auto = false;
            }
            return;
        }

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

    // Custom methods
    public boolean isBusy() {
        return (driveLeft.isBusy() || driveRight.isBusy());
    }

    public boolean isDone() {
        return((Math.abs(driveLeft.getCurrentPosition()) >= Math.abs(driveLeft.getTargetPosition())) ||
                (Math.abs(driveRight.getCurrentPosition()) >= Math.abs(driveRight.getTargetPosition())));
    }

    public DcMotor.RunMode LdriveMode() {
        return driveLeft.getMode();
    }
    public DcMotor.RunMode RdriveMode() {
        return driveRight.getMode();
    }

    public double leftDrivePos() {
        return (driveLeft.getCurrentPosition() / TICKS_PER_INCH);
    }

    public double rightDrivePos() {
        return (driveRight.getCurrentPosition() / TICKS_PER_INCH);
    }

    public void driveTo(float speed, float distance) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
        }

        // Set a target, translated from inches to encoder ticks
        int leftTarget = driveLeft.getCurrentPosition();
        int rightTarget = driveRight.getCurrentPosition();
        leftTarget += distance * TICKS_PER_INCH;
        rightTarget += distance * TICKS_PER_INCH;
        driveLeft.setTargetPosition(leftTarget);
        driveRight.setTargetPosition(rightTarget);

        // Start the motors
        auto = true;
        driveLeft.setPower(speed);
        driveRight.setPower(speed);
    }

    public void turnTo(float speed, int angle) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
        }

        // Fake turns using a distance translation
        // We have a gyro but let's start with just one control mode
        int leftTarget = driveLeft.getCurrentPosition();
        int rightTarget = driveRight.getCurrentPosition();
        leftTarget += angle * TURN_RATIO;
        rightTarget -= angle * TURN_RATIO;
        driveLeft.setTargetPosition(leftTarget);
        driveRight.setTargetPosition(rightTarget);

        // Start the motors
        auto = true;
        driveLeft.setPower(speed);
        driveRight.setPower(-speed);
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeft.setTargetPosition(driveLeft.getCurrentPosition());
        driveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeft.setPower(0);

        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setTargetPosition(driveRight.getCurrentPosition());
        driveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRight.setPower(0);
    }
}
