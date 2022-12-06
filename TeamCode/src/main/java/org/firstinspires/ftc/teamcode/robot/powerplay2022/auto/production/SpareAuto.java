package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import android.graphics.Color;

/*
 * This routine runs a standard autonomous route based upon cone color 
 * for the spare robot running Neverest motors
 */

public class SpareAuto extends DistanceDemoBase {

    Direction direction;

    public SpareAuto() {
    }

    @Override
    public void runOpMode( int _colorVal, String _colorText ) throws InterruptedException {

        int encoderDrivingTarget= 0;

        initializePower( 0.3 );

        setVerticalTicksPerInch( 2000 / 46.75 );

        setLateralTicksPerInch( 2000 / 40.5 );

        // move forward to read the signal sleeve
        encoderDrivingTarget = calculateDistance( 17.0, Direction.FORWARD );

        driveTo( Direction.FORWARD, encoderDrivingTarget );
        moveTo( power );

        telemetry.addData("Drive to Cone and Read Color:",_colorText);
        telemetry.update();
        sleep( 3000 );

        // move forward to the stack of 5 cones
        encoderDrivingTarget = calculateDistance( 31.0, Direction.FORWARD );

        driveTo( Direction.FORWARD, encoderDrivingTarget );
        moveTo( power );

        if (_colorVal == Color.CYAN) {
            direction = Direction.RIGHT;
        } else if (_colorVal == Color.MAGENTA) {
            direction = Direction.LEFT;
        } else {
            direction = null;
        }

        if (direction!=null) {

            // move laterally based upon cone color
            encoderDrivingTarget = calculateDistance(23.0, direction);

            driveTo( direction, encoderDrivingTarget );
            moveTo(power);
        }
    }
}
