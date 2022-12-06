package org.firstinspires.ftc.teamcode.drives.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.sensors.pot.BasicPotentiometer;
import org.firstinspires.ftc.teamcode.utils.sensors.pot.DoublePotentiometer;

@Disabled
@TeleOp(name = "SwerveTUNE", group = "test")
public class SwerveAutoTune extends LinearOpMode {
    private BasicPotentiometer pot = null;
    private BasicPotentiometer pot2 = null;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry t = dash.getTelemetry();

    private double min = 0.5;
    private double max = 0.5;

    private SwervePod pod1;

    @Override
    public void runOpMode() {
        telemetry.addData("status", "initing . . .");

        try{
            pod1 = new SwervePod(t, hardwareMap.get(DcMotor.class, "p1m1"), hardwareMap.get(DcMotor.class, "p1m2"), DoublePotentiometer.FromData(hardwareMap, t, "pot1a", "pot1b", 90));
        } catch (Exception e) {
            telemetry.log().add("failed to init");
        }
        RobotLog.d("," +  + getRuntime() + "," + "initied swerve autotune");

        waitForStart();

        //run tuning
        //pod1.tune();

        //does tpr make sense?
        //pod1.hmmm(1740);


        //pod1.hmmmm();

        while(opModeIsActive()){
            //linearization working?
                pod1.spin();
        }
    }
}
