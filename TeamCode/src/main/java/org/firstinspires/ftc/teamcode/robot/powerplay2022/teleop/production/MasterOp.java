package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production.MecanumAuto;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Disabled
@Autonomous(name = "MasterOp", group = "A")
public class MasterOp extends LoopUtil {

    public static LoopUtil auto, tele;
    public double totalTime;
    public static CustomMecanumDrive mecanumDrive;
    public static InputHandler gamepadHandler;

    @Override
    public void opInit() {
        tele = new ActualTeleOp();
        auto = new MecanumAuto();

        tele.hardwareMap = this.hardwareMap;
        auto.hardwareMap = this.hardwareMap;
        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        tele.gamepad1 = this.gamepad1;
        auto.gamepad1 = this.gamepad1;
        tele.gamepad2 = this.gamepad2;
        auto.gamepad2 = this.gamepad2;
        tele.telemetry = this.telemetry;
        auto.telemetry = this.telemetry;

        tele.opInit();
        auto.opInit();

        mecanumDrive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);

        //Go-Builda motor mapping:
        mecanumDrive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true, false);

        //testing mecanum mapping
        //mecanumDrive.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);

        //sets the sub opmodes' drive to this master drive
        DriveTest.drive = mecanumDrive;
        MecanumAuto.drive = mecanumDrive;

        totalTime = 0;
    }

    @Override
    public void opInitLoop() {
        auto.opInitLoop();
    }

    @Override
    public void opStart() {
    }

    @Override
    public void opUpdate(double deltaTime) {


        totalTime+=deltaTime;
        if(totalTime < 30 * EULConstants.SEC2MS){
            auto.opUpdate(deltaTime);
        }else{
            tele.opFixedUpdate(deltaTime);
            tele.opUpdate(deltaTime);
        }
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
