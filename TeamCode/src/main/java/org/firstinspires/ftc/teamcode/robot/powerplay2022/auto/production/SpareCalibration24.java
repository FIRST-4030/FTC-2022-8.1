package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
* This is a sample routine to show how to move the robot 24 inches forward.
 */

@Disabled
@Autonomous(name = "SpareCalibration24", group = "actual")
public class SpareCalibration24 extends DistanceDemoBase {

    public SpareCalibration24() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        initializePower( 0.3 );

        setVerticalTicksPerInch( 2000 / 46.75 );

        waitForStart();

        int encoderDrivingTarget = calculateDistance( 24.0, Direction.FORWARD );

        telemetry.addData("Ticks Needed:", encoderDrivingTarget);
        telemetry.update();
        sleep( 3000 );

        driveTo( Direction.FORWARD, encoderDrivingTarget );
        moveTo( power );
        sleep( 3000 );
    }
}
