package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.controllers.NormalizedController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.examples.MotorPIDCommand;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

public class LinearSlide {

    private ConditionalManager stateMachine;
    private final NormalizedController normalizedController;

    private final DcMotorEx motorEx;

    private final int reversal;

    public LinearSlide(HardwareMap hardwareMap, String name, boolean reverseMotor){
        this.motorEx = (DcMotorEx) hardwareMap.dcMotor.get(name);
        this.reversal = reverseMotor ? -1 : 1;

        this.normalizedController = new NormalizedController(0.5, 0.1, 0.4, 0.1, 0.7);
        this.normalizedController.setMaxExpectedError(75);

        this.stateMachine = new ConditionalManager();
    }

    public void seek(double target){
        this.motorEx.setPower(this.normalizedController.seek(target, motorEx.getCurrentPosition()) * reversal);
        if (Math.abs(target - motorEx.getCurrentPosition()) <= 5.0){
            this.motorEx.setPower(0);
        }
    }

    public void queue(double target) {
        this.stateMachine.addConditional(
                new MotorPIDCommand(
                        this.normalizedController.getKp(),
                        this.normalizedController.getKi(),
                        this.normalizedController.getKd(),
                        motorEx)
                .setTarget(target));
    }

    public void runStates(){
        this.stateMachine.execute();
    }

    public static class PowerPlayLS extends LinearSlide{
        public PowerPlayLS(HardwareMap hardwareMap, String name, boolean reverseMotor) {
            super(hardwareMap, name, reverseMotor);
        }

        public void setPosition(int pos){
            if (pos == 3){
                super.seek(540d / 3 + 290);
            } else if (pos == 1){
                super.seek(540d / 3 - 50);
            } else if (pos == 2){
                super.seek(540d / 3 + 235);
            } else {
                super.seek(0);
            }
        }
    }
}
