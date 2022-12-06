package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager;

public abstract class Conditional {

    public static Conditional DEFAULT = new Conditional() {
        @Override
        public void init() {
            linkedStates = new int[0];
        }

        @Override
        public void check() {
            status = STATUS.FAILED;
        }
    };

    public enum STATUS{
        FAILED,
        PASSED
    }

    public STATUS status;
    public int[] linkedStates;

    public abstract void init();

    public abstract void check();
}