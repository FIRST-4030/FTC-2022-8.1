package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
* This is a very crude attempt at showing how to use encoder
* ticks to move the robot in a very controlled way.
*
* The robot moves forward and backward and then left and right.
 */

@Disabled
@Autonomous(name = "DirectionDemo", group = "actual")
public class DirectionDemo extends DistanceDemoBase {

    public DirectionDemo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        initializePower( 0.3 );

        waitForStart();

        int encoderDrivingTarget = 1000;

        driveTo( Direction.FORWARD, encoderDrivingTarget );
        moveTo( power );

        driveTo( Direction.REVERSE, encoderDrivingTarget );
        moveTo( power );

        driveTo( Direction.RIGHT, encoderDrivingTarget );
        moveTo( power );

        driveTo( Direction.LEFT, encoderDrivingTarget );
        moveTo( power );
    }
}
