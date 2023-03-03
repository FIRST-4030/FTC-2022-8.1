package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

public abstract class Conditional {

    public static Conditional DEFAULT = new Conditional() {
        @Override
        public void init() {

        }

        @Override
        public void setupStates(int[] linkedStates) {

        }

        @Override
        public void execute() {

        }
        @Override
        public void end() {

        }

        @Override
        public boolean isFinished() {
            return true;
        }
    };
    public int[] linkedStates;

    public abstract void init();
    public abstract void setupStates(int[] linkedStates);
    public abstract void execute();
    public abstract void end();

    public abstract boolean isFinished();

    public void bindManager(ConditionalManager manager){}
}