package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.examples;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

public class ConditionalManagerExampleOpMode extends EnhancedOpMode {

    private ConditionalManager stateMachine;
    private InputHandler controllers;

    private DcMotorEx slideMotor;
    private MotorPIDCommand slideMotorPIDCommand;

    @Override
    public void opInit() {
        controllers = InputAutoMapper.normal.autoMap(this);
        stateMachine = new ConditionalManager();

        slideMotor = (DcMotorEx) hardwareMap.dcMotor.get("LSRM");
        slideMotorPIDCommand = new MotorPIDCommand(0.6, 0, 0.4, slideMotor);
    }

    @Override
    public void opSetupControllers(DSController driver1, DSController driver2) {
        driver1.buttonA.bindConditional(DSController.Digital.FLAG.ON_RELEASE, slideMotorPIDCommand.setTarget(0));
        driver1.buttonB.bindConditional(DSController.Digital.FLAG.ON_RELEASE, slideMotorPIDCommand.setTarget(540d / 3 - 50));
        driver1.buttonX.bindConditional(DSController.Digital.FLAG.ON_RELEASE, slideMotorPIDCommand.setTarget(540d / 3 + 235));
        driver1.buttonY.bindConditional(DSController.Digital.FLAG.ON_RELEASE, slideMotorPIDCommand.setTarget(540d / 3 + 290));
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opInput(double deltaTimeMS) {

    }

    @Override
    public void opUpdate(double deltaTimeMS) {
        //log FPS and delta time per loop
        telemetry.addData("FPS: ", getFps());
        telemetry.addData("Delta Time (SEC): ", getDeltaTimeSEC());
        telemetry.addData("Delta Time (MS): ", getDeltaTimeMS());
        telemetry.addData("Delta Time (NANO): ", getDeltaTimeNANO());
    }

    @Override
    public void opFixedUpdate(double deltaTimeMS) {

    }

    @Override
    public void opStop() {

    }
}
