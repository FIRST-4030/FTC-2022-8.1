package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement;

import org.firstinspires.ftc.teamcode.utils.general.misc.VirtualRobot;

public class MecanumDriveState {

    public interface Conditional{
        boolean isDone();
        void update(VirtualRobot virtualRobot);
        String getName();
    }

    public static class TimeCondition implements Conditional{
        public double elapsedTime;
        public double targetTime;

        public TimeCondition(double targetTime){
            this.targetTime = targetTime;
            elapsedTime = 0;
        }

        @Override
        public boolean isDone() {
            return elapsedTime >= targetTime && targetTime != -1;
        }

        @Override
        public void update(VirtualRobot virtualRobot) {
            elapsedTime += virtualRobot.getDeltaTime();
        }
        @Override
        public String getName() {
            return getClass().getSimpleName();
        }
    }

    public static class HeadingCondition implements Conditional{

        public double target;
        public double actual;
        public double marginOfError;

        public HeadingCondition(double target, double marginOfError){
            this.target = target;
            this.actual = -1;
            this.marginOfError = marginOfError;
        }

        @Override
        public boolean isDone() {
            return (Math.abs(target - actual) <= marginOfError);
        }

        @Override
        public void update(VirtualRobot virtualRobot) {
            actual = virtualRobot.getHeadingAngle();
        }

        @Override
        public String getName() {
            return getClass().getSimpleName();
        }
    }

    public String name;
    public Runnable state;
    public Conditional conditional;
    public MecanumDriveState(String name, Runnable state, Conditional conditional){
        this.name = name;
        this.state = state;
        this.conditional = conditional;
    }

    public void update(VirtualRobot vr){
        state.run();
        conditional.update(vr);
    }

    public boolean isDone(){
        return conditional.isDone();
    }
}
