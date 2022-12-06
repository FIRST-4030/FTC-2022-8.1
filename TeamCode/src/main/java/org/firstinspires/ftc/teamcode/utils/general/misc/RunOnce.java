package org.firstinspires.ftc.teamcode.utils.general.misc;

public abstract class RunOnce implements Runnable{

    private boolean latch = false;

    public void update(){
        if (!latch){
            run();
            latch = true;
        }
    }

    public void toggleLatch(){
        latch = !latch;
    }

    public abstract void run();
}
