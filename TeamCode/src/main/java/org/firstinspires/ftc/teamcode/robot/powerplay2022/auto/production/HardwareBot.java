package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBot {
    /* Define hardware */
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;

    HardwareMap hwMap;

    public HardwareBot() {
    }

    public void init(HardwareMap _hwMap) {
        hwMap = _hwMap;

        /* Define hardware */
        backLeft = hwMap.get(   DcMotor.class, "BL" );
        backRight = hwMap.get(  DcMotor.class, "BR" );
        frontLeft = hwMap.get(  DcMotor.class, "FL" );
        frontRight = hwMap.get( DcMotor.class, "FR" );

        backLeft.setDirection(   DcMotor.Direction.REVERSE );
        backRight.setDirection(  DcMotor.Direction.REVERSE );
        frontLeft.setDirection(  DcMotor.Direction.FORWARD );
        frontRight.setDirection( DcMotor.Direction.FORWARD );

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setPower( 0 );
        backRight.setPower( 0 );
        frontLeft.setPower( 0 );
        frontRight.setPower( 0 );
    }
}
