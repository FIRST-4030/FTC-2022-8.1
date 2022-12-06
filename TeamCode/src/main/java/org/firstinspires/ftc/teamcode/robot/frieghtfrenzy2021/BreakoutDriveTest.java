package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;

@Config
@Disabled
@TeleOp(name = "BreakoutDriveTest", group = "Test")
public class BreakoutDriveTest extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final double MAX_VELOCITY = 40.0; // inches per second
    public static double TICKS_PER_INCH = 40.75;
    public static double trackWidth = 15.25;
    public static double trackWidthHalf = trackWidth / 2.0;
    private double leftPower = 0;
    private double rightPower = 0;
    private boolean leftSide = false;
    private static double INCREMENT = 0.01;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private InputHandler in;
    private boolean loggingEnabled = false;

    // Standard methods
    @Override
    public void init() {
        // Drive wheels
        try {
            driveLeft = hardwareMap.get(DcMotor.class, "BL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            driveRight = hardwareMap.get(DcMotor.class, "BR");
            driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Globals.opmode = this;
            in = Globals.input(this);
            in.register("+", GAMEPAD.driver1, PAD_KEY.dpad_up);
            in.register("-", GAMEPAD.driver1, PAD_KEY.dpad_down);

            enabled = true;

        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
        loggingEnabled = false;
        logDataInit();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        leftPower = 0;
        rightPower = 0;
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        if (gamepad1.dpad_left) leftSide = true;
        if (gamepad1.dpad_right) leftSide = false;

        telemetry.addData("Drive Side: ", leftSide ? "Left" : "Right");

        // Input
        in.loop();
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        if (leftSide) {
            // Moving the servo position and number should increase
            if (in.down("+")) {
                leftPower += INCREMENT;
                leftPower = Math.min(1.0, leftPower);
            // Moving the servo position and number should decrease
            } else if (in.down("-")) {
                leftPower -= INCREMENT;
                leftPower = Math.max(-1.0, leftPower);
            }
        } else {
            // Moving the servo position and number should increase
            if (in.down("+")) {
                rightPower += INCREMENT;
                rightPower = Math.min(1.0, rightPower);
            // Moving the servo position and number should decrease
            } else if (in.down("-")) {
                rightPower -= INCREMENT;
                rightPower = Math.max(-1.0, rightPower);
            }
        }
        driveLeft.setPower(leftPower);
        driveRight.setPower(rightPower);
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
        super.stop();
    }

    // Custom methods
    public boolean isBusy() {
        return (driveLeft.getPower() != 0 || driveRight.getPower() != 0);
    }

    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }

    public void enableLogging() {
        loggingEnabled = true;
    }

    public void disableLogging() {
        loggingEnabled = false;
    }

    /**
     * Initialize the headers for all of the logged data
     * Note that this does NOT add headers for any function-specific data
     */
    private void logDataInit() {
        RobotLog.d("");
        RobotLog.d(",Drive Function,Time (s),Left Position (in),Right Position (in),Left Velocity (in/s),Right Velocity (in/s)");
    }

    /**
     * Log the basic drive data to a text file. Also log the "additionalData" string;
     */
    private void logData(String functionName, String additionalData) {
        if (loggingEnabled)
            RobotLog.d("," + functionName + "," + getRuntime() + "," +
                    driveLeft.getCurrentPosition() / TICKS_PER_INCH + "," + driveRight.getCurrentPosition() / TICKS_PER_INCH + "," +
                    driveLeft.getPower() * MAX_VELOCITY + "," + driveRight.getPower() * MAX_VELOCITY + "," +
                    additionalData);
    }
}
