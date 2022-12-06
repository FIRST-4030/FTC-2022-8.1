package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This routine runs a standard autonomous mode after reading MAGENTA on the signal sleeve
 */

@Disabled
@Autonomous(name = "SpareAutoMagenta", group = "actual")
public class SpareAutoMagenta extends SpareAuto {

    public SpareAutoMagenta() throws InterruptedException {
    }

    public void runOpMode() throws InterruptedException {

        robot.init( hardwareMap );

        waitForStart();

        runOpMode( Color.MAGENTA, "Magenta" );
    }
}
