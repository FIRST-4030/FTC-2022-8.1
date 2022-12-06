package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This class provides methods for moving the robot based upon an encoder.
 */

public class DistanceDemoBase extends LinearOpMode {

    enum Direction {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    }

    static final double STOP  = 0.0;

    double power = 0.0;
    double verticalTicksPerInch = 0;
    double lateralTicksPerInch = 0;

    HardwareBot robot = new HardwareBot();

    public DistanceDemoBase() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void runOpMode( int _colorVal, String _colorText ) throws InterruptedException {
    }

    public int calculateDistance( double _distance, Direction _dir ) {

        if (_dir==Direction.FORWARD || _dir==Direction.REVERSE) {
            return (int)(_distance * verticalTicksPerInch);
        }
        else { // (_dir==Direction.LEFT || _dir==Direction.RIGHT)
            return (int)(_distance * lateralTicksPerInch);
        }
    }

    public void initializePower( double _power ) {
        power = _power;
    }

    public void driveTo( Direction _dir, int _distance ) {

        if (_dir==Direction.FORWARD || _dir==Direction.REVERSE) {

            int absDistance = _distance;
            if (_dir == Direction.REVERSE) {
                absDistance = -absDistance;
            }

            robot.frontLeft.setTargetPosition(    absDistance  );
            robot.frontRight.setTargetPosition( -(absDistance) );
            robot.backLeft.setTargetPosition(   -(absDistance) );
            robot.backRight.setTargetPosition(    absDistance  );
        }
        else { // (_dir==Direction.LEFT || _dir==Direction.RIGHT)

            int absDistance = _distance;
            if (_dir == Direction.LEFT) {
                absDistance = -absDistance;
            }

            robot.frontLeft.setTargetPosition(  absDistance );
            robot.frontRight.setTargetPosition( absDistance );
            robot.backLeft.setTargetPosition(   absDistance );
            robot.backRight.setTargetPosition(  absDistance );
        }
    }

    public void moveTo( double _power ) {

        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        setMode( DcMotor.RunMode.RUN_TO_POSITION );

        setPower( _power );

        while ( robot.backLeft.isBusy()  ||
                robot.backRight.isBusy() ||
                robot.frontLeft.isBusy() ||
                robot.frontRight.isBusy() ) {
            telemetry.addData("Back Left:", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left:", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", robot.frontRight.getCurrentPosition());
            telemetry.update();
        }
        setPower( STOP );
    }

    public void setMode( DcMotor.RunMode _mode ) {
        robot.backLeft.setMode(   _mode );
        robot.backRight.setMode(  _mode );
        robot.frontLeft.setMode(  _mode );
        robot.frontRight.setMode( _mode );
    }

    public void setPower( double _power ) {
        robot.backLeft.setPower(   _power );
        robot.backRight.setPower(  _power );
        robot.frontLeft.setPower(  _power );
        robot.frontRight.setPower( _power );
    }

    public void setVerticalTicksPerInch( double _ticks ) { verticalTicksPerInch = _ticks; }

    public void setLateralTicksPerInch( double _ticks ) { lateralTicksPerInch = _ticks; }
}
