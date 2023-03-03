package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.examples;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.pidcontroller.controllers.NormalizedController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions.Conditional;

public class MotorPIDCommand extends Conditional {

    private DcMotorEx motor;
    private NormalizedController pid;

    private double target;

    private double tickTolerance;

    public MotorPIDCommand(double kP, double kI, double kD, DcMotorEx motor){
        this.motor = motor;
        this.pid = new NormalizedController(kP, kI, kD);
        this.tickTolerance = 2;
    }

    public NormalizedController getPID(){
        return this.pid;
    }

    public Conditional setTarget(double target){
        this.target = target;
        return this;
    }

    @Override
    public void init() {
        pid.reset();
    }

    @Override
    public void setupStates(int[] linkedStates) {

    }

    @Override
    public void execute() {
        motor.setPower(pid.seek(target, motor.getCurrentPosition()));
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return target - motor.getCurrentPosition() <= tickTolerance;
    }
}
