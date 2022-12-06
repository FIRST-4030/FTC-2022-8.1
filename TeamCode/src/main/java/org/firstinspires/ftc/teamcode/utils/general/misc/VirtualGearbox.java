package org.firstinspires.ftc.teamcode.utils.general.misc;

public class VirtualGearbox {

    private int inputTeeth;
    private int outputTeeth;

    private double ratio;
    private double inverseRatio;

    public VirtualGearbox(int firstTeeth, int secondTeeth){
        this.inputTeeth = firstTeeth;
        this.outputTeeth = secondTeeth;
        this.ratio = ((double) inputTeeth) / ((double) outputTeeth);
        this.inverseRatio = ((double) outputTeeth) / ((double) inputTeeth);
    }

    public double toOutput(double in){
        return in * ratio;
    }

    public int getFirstGearTeeth(){
        return inputTeeth;
    }

    public int getSecondGearTeeth(){
        return outputTeeth;
    }

    public double getRatio(){
        return ratio;
    }

    public double getInverseRatio(){
        return inverseRatio;
    }
}
