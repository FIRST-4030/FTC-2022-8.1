package org.firstinspires.ftc.teamcode.utils.threadingUtils;

import org.firstinspires.ftc.teamcode.utils.general.misc.CSASV;

public abstract class MRUTask implements Runnable{

    //mimic making separate static value for each child class
    private static CSASV<Boolean>
            isDone = new CSASV<>(),
            isRunning = new CSASV<>();

    private static long UPDATE_CAP = 1/30;

    private Class staticAllocator = getClass();

    public MRUTask(){
        isDone.register(this, true);
        isRunning.register(this, false);
    }

    public abstract void init();
    public abstract void update(double deltaTime);
    public abstract void fixed_update(double deltaTime);

    @Override
    public void run() {
        double
                start,
                end = 0,
                delta,
                unprocessed = 0,
                frameTime = 0,
                frames = 0,
                fps = 0;

        while(getIsRunning()) {
            start = System.currentTimeMillis();
            delta = start - end;
            end = start;

            unprocessed += delta;

            setIsDone(false);

            while (unprocessed >= UPDATE_CAP) {
                //everytime this loops, it will try to update as many times as what a perfect loop will update every second;
                //this will limit the loop so it doesn't create an infinite loop and stall out the OpMode
                unprocessed -= UPDATE_CAP;

                if (frameTime >= 1.0) {
                    //extra for logging additional data
                    frameTime = 0;
                    fps = frames;
                    frames = 0;

                    fixed_update(delta);
                }
                update(delta);
            }

            setIsDone(true);
        }
    }


    //start of the child static stuff
    private void setIsDone(boolean doneState){
        isDone.set(this, doneState);
    }

    private boolean getIsDone(){
        return isDone.get(this);
    }

    public static boolean isDone(Class childClass){
        return isDone.get(childClass);
    }

    private void setIsRunning(boolean runningState){
        isRunning.set(this, runningState);
    }

    private boolean getIsRunning(){
        return isRunning.get(this);
    }

    public static boolean isRunning(Class childClass){
        return isRunning.get(childClass);
    }
    //end of child static variable shenanigans


    private void waitMs(long msWaitTime){
        long timeStart = System.currentTimeMillis();
        long delta = timeStart;
        while(delta < msWaitTime){
            delta = System.currentTimeMillis() - timeStart;
        }
    }

    private void waitNano(long nanoWaitTime){
        long timeStart = System.nanoTime();
        long delta = timeStart;
        while (delta < nanoWaitTime){
            delta = System.nanoTime() - timeStart;
        }
    }
}
