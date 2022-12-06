package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@Autonomous(name = "TestNewDrive", group = "Test")
public class NewTestDrive extends OpMode {

    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
    public static double TURN_RATIO = 7;
    public static double ACCEL_CONSTANT = 0.4;
    public static double LDThreshold = 0.5, RDThreshold = 0.5;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    //Members
    private double currentLDPos = 0, currentRDPos = 0; //this needs to be updated every loop
    private double targetLDPos = 0, targetRDPos; //this will be the target pos, will need to update evey call of turnTo & driveTo
    private double powerLD = 0, powerRD = 0;

    @Override
    public void init() {
        driveLeft = hardwareMap.get(DcMotor.class, "BL");
        driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveRight = hardwareMap.get(DcMotor.class, "BR");
        driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    int num = 0;
    @Override
    public void loop() {
        telemetry.addData("isBusy(): ", this.isBusy());
        switch (num){
            case 1:
                this.driveTo(0.4, 10);
                if (!this.isBusy()) {num++;}
                break;
            case 2:
                this.turnTo(0.4, 90);
                if (!this.isBusy()) {num++;}
                break;
            case 3:
                this.driveTo(0.4, 3);
                if (!this.isBusy()) {num++;}
                break;
            default:
                break;
        }
    }

    public void update(){
        //update current position of each motor
        this.currentLDPos = this.driveLeft.getCurrentPosition();
        this.currentRDPos = this.driveRight.getCurrentPosition();

        //update target position of each motor
        this.targetLDPos = this.driveLeft.getTargetPosition();
        this.targetRDPos = this.driveRight.getTargetPosition();

        this.powerLD = this.driveLeft.getPower();
        this.powerRD = this.driveRight.getPower();
    }

    public boolean isBusy(){
        this.update();
        double diffL = Math.abs(this.targetLDPos - this.currentLDPos);
        double diffR = Math.abs(this.targetRDPos - this.currentRDPos);

        return !((diffL <= LDThreshold * TICKS_PER_INCH) && (diffR <= RDThreshold * TICKS_PER_INCH));
    }

    public void driveTo(double speed, double distance){
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
            return;
        }

        // Set a target, translated from inches to encoder ticks
        int leftTarget = driveLeft.getCurrentPosition();
        int rightTarget = driveRight.getCurrentPosition();
        leftTarget += distance * TICKS_PER_INCH;
        rightTarget += distance * TICKS_PER_INCH;
        driveLeft.setTargetPosition(leftTarget);
        driveRight.setTargetPosition(rightTarget);

        // Start the motors
        driveLeft.setPower(speed);
        driveRight.setPower(speed);

        this.update();
    }

    public void turnTo(double speed, double angle){
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
            return;
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
        driveLeft.setPower(speed);
        driveRight.setPower(-speed);

        this.update();
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
