package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import java.util.ArrayList;
//TODO: Add documentation
public class EncoderTickBuffer {

    public ArrayList<Double> rawValues;
    public ArrayList<Integer> approxValues;
    public ArrayList<Double> powerPartition;
    protected double tickError;
    protected int cursor;

    public EncoderTickBuffer(){
        this.rawValues = new ArrayList<>();
        this.approxValues = new ArrayList<>();
        this.powerPartition = new ArrayList<>();

        rawValues.add(0d);
        approxValues.add(0);
        tickError = 0;
        cursor = 0;
    }
    public EncoderTickBuffer(ArrayList<Double> existingRaws, ArrayList<Double> existingPowerPartitions){
        this.rawValues = existingRaws;
        this.approxValues = new ArrayList<>();
        this.powerPartition = existingPowerPartitions;

        rawValues.add(0d);
        approxValues.add(0);
        tickError = 0;
        cursor = 0;
    }

    public EncoderTickBuffer addRawValue(double tick, double power){
        this.rawValues.add(tick);
        this.powerPartition.add(power);
        return this;
    }

    public void buildApprox(){
        double currentValue;
        int approxValue, sign;
        for (int i = 0; i < rawValues.size(); i++){ //literally a piece of Bresenham's line drawing algorithm for plotting pixels
            currentValue = rawValues.get(i);
            approxValue = (int) currentValue;
            sign = (int) Math.signum(currentValue - approxValue);
            tickError += currentValue - approxValue;
            if (tickError >= 0.5){
                approxValues.add(approxValue + sign);
                tickError += sign;
            } else {
                approxValues.add(approxValue);
            }
        }
    }

    public EncoderTickBuffer makeReverse(){
        EncoderTickBuffer outputBuffer = new EncoderTickBuffer();
        for (int i = 0; i < rawValues.size(); i++) {
            outputBuffer.addRawValue(rawValues.get(rawValues.size() - 1 - i), powerPartition.get(rawValues.size() - 1 - i));
        }
        outputBuffer.buildApprox();

        return outputBuffer;
    }
}
