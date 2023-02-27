package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions;

import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.managers.ConditionalManager;

public abstract class Conditional {

    public static Conditional DEFAULT = new Conditional() {
        @Override
        public void init() {

        }

        @Override
        public void setupStates(int[] linkedStates) {
            linkedStates = new int[0];
        }

        @Override
        public void execute() {

        }
        @Override
        public void end() {

        }

        @Override
        public boolean isFinished() {
            return false;
        }
    };
    public int[] linkedStates;

    public abstract void init();
    public abstract void setupStates(int[] linkedStates);
    public abstract void execute();
    public abstract void end();

    public abstract boolean isFinished();

    public void bindManager(ConditionalManager manager){

    }
}