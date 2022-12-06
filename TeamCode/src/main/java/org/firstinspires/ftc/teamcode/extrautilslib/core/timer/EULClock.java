package org.firstinspires.ftc.teamcode.extrautilslib.core.timer;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;

public class EULClock {

    private long startTime = 0, deltaTime = 0, stopTime = 0;
    private boolean stopped = true;

    public EULClock(){}

    public void start(){
        this.stopped = false;
        this.startTime = System.nanoTime();
        this.stopTime = this.startTime;
    }

    public void stop(){
        this.stopped = true;
        this.stopTime = System.nanoTime();
        this.deltaTime = this.stopTime - this.startTime;
    }

    public long getStartTime(){return this.startTime;}
    public long getDeltaTime(){return this.stopped ? this.deltaTime : getElapsedTime();}
    public long getStopTime(){return this.stopped ? this.stopTime : getElapsedTime();}
    public long getElapsedTime(){return System.nanoTime() - this.startTime;}
    public boolean isStopped(){return this.stopped;}

    public void waitFor(long ms){
        double ns = ms * EULConstants.MS2NANO;
        long start = System.nanoTime();
        while (System.nanoTime() - start < ns){}
    }
}
