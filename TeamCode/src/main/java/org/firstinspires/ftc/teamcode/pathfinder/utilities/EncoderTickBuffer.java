package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import java.util.ArrayList;
//TODO: Add documentation
public class EncoderTickBuffer {

    public ArrayList<Double> rawValues;
    public ArrayList<Integer> approxValues;
    protected double tickError;

    public EncoderTickBuffer(){
        this.rawValues = new ArrayList<>();
        this.approxValues = new ArrayList<>();

        rawValues.add(0d);
        approxValues.add(0);
        tickError = 0;
    }

    public EncoderTickBuffer addRawValue(double tick){
        this.rawValues.add(tick);
        return this;
    }

    public void buildApprox(){
        double currentValue;
        for (int i = 0; i < rawValues.size(); i++){ //literally a piece of Bresenham's line drawing algorithm
            currentValue = rawValues.get(i);
            tickError += currentValue - ((int) currentValue);
            if (tickError >= 0.5){
                approxValues.add((int) currentValue + 1);
                tickError--;
            } else {
                approxValues.add((int) currentValue);
            }
        }
    }
}
