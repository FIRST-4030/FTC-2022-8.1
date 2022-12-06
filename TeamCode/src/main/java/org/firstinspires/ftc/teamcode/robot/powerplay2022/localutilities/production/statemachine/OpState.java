package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.statemachine;

import java.util.ArrayList;
import java.util.Arrays;

public class OpState {
    public ArrayList<Runnable> asyncRunnable;
    public double elapsedTime = 0;

    public OpState(){
        this.asyncRunnable = new ArrayList<>();
    }

    public OpState(Runnable... runnable) {
        this.asyncRunnable = new ArrayList<>();
        this.asyncRunnable.addAll(Arrays.asList(runnable));
    }

    public void runAll(double dt){
        elapsedTime += dt;
        for (Runnable runnable: asyncRunnable) {
            runnable.run();
        }
    }
}
