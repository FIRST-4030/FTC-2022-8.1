package org.firstinspires.ftc.teamcode.utils.general.misc;

public abstract class LoopOnce implements Runnable{
    private boolean ran = false;

    public void run(){
        if (!this.ran){
            loop();
            this.ran = true;
        }
    }

    public abstract void loop();

    public boolean hasRan(){
        return this.ran;
    }

    public void reset(){
        this.ran = false;
    }
}
