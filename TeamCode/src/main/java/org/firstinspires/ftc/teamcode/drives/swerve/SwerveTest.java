package org.firstinspires.ftc.teamcode.drives.swerve;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.sensors.pot.DoublePotentiometer;

@Disabled
@TeleOp(name = "SwerveTest", group = "Test")
public class SwerveTest extends LinearOpMode {

    private SwervePod pod1;
    private SwervePod pod2;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry t = dash.getTelemetry();

    private final double turnCoef = 0.2;


    @Override
    public void runOpMode() {
        telemetry.addData("status", "initing . . .");
        //Hardware map initializing
        try {
            pod1 = new SwervePod(t, hardwareMap.get(DcMotor.class, "p1m1"), hardwareMap.get(DcMotor.class, "p1m2"), DoublePotentiometer.FromData(hardwareMap, t, "pot1a", "pot1b", 90));
          //  pod2 = new SwervePod(t, hardwareMap.get(DcMotor.class, "p2m1"), hardwareMap.get(DcMotor.class, "p2m2"), DoublePotentiometer.FromData(hardwareMap, t, "pot2a", "pot2b", 90));
        } catch (Exception e) {
            telemetry.log().add("failed to init");
        }

        waitForStart();

        //Main loop
        for (; opModeIsActive(); ) {
            //angle from joystick
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);

            t.addData("input", angle);

            //drives slow to better be able to tell what is going on
            double main_velocity = 0.4*Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
            double turn = gamepad1.right_stick_x * turnCoef;

            if (turn == 0) {
                if (main_velocity != Double.NaN) {
                    //if not turning point wheels in dirrection of stick
                    pod1.setTargetAngle(angle);
                  //  pod2.setTargetAngle(angle);
                } else {
                    //no inputs make sure things are zerod
                    pod1.zero();
                   // pod2.zero();
                }
            } else {
                //parallel wheels
                pod1.setTargetAngle(Math.PI/2);
                //pod2.setTargetAngle(Math.PI/2);
            }
            pod1.setTargetVelocity(main_velocity+turn);
            //pod2.setTargetVelocity(main_velocity-turn);

            //update pods
            pod1.loop();
           // pod2.loop();

            //update telemtry
            t.update();
        }
    }
}
