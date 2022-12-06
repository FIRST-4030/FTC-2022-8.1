package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.pathfinder.control.PathFinderDrive;

import java.util.Vector;

public class PFPath {

    private PathFinderDrive drive;
    protected Vector<float[]> encoderValues;
    protected Vector<float[]> powerValues;
    protected Vector<PFPose2d> poseLookup;
    protected int idx;

    public PFPath(PathFinderDrive drive, PFPose2d initialPose){
        this.encoderValues = new Vector<>();
        this.poseLookup = new Vector<>();
        this.idx = 0;

        this.poseLookup.add(initialPose);
        this.drive = drive;
    }

    public void updatePathProgress(float... realEncoderValues){
        float[] previousValues = new float[realEncoderValues.length];
        float[] targetValues = new float[previousValues.length];
        float averageProgress = 0;

        for (int i = 0; i < realEncoderValues.length; i++) {
            previousValues[i] = idx > 0 ? encoderValues.get(i)[idx - 1] : 0;
            targetValues[i] = encoderValues.get(i)[idx];

            averageProgress += (realEncoderValues[i] - previousValues[i]) / (targetValues[i] - previousValues[i]);
        }

        averageProgress /= realEncoderValues.length;

        if (averageProgress >= 1.0f) idx++;
    }

    public Vector<PFPose2d> getPoseLookup(){
        return this.poseLookup;
    }

    public void build(){
        this.drive.buildPath(this);
    }

    public void setEncoderValues(Vector<float[]> nValues) {
        this.encoderValues = nValues;
    }

    public float[] getCurrentEncoderValues(){
        return this.encoderValues.get(idx);
    }

    public void setPowerValues(Vector<float[]> nValues){
        this.powerValues = nValues;
    }

    public float[] getCurrentPowerValues(){
        return this.powerValues.get(idx);
    }

    public PFPath advance(double dist){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //exploit directional information to find the new pose's position
        this.poseLookup.add(new PFPose2d(prev.pos.plus(prev.getDir().times(dist)), prev.heading));
        return this;
    }

    public PFPath strafe(double dist){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //exploit tangential information to find the new pose's position
        this.poseLookup.add(new PFPose2d(prev.pos.plus(prev.getNormal().times(dist)), prev.heading));
        return this;
    }

    public PFPath turn(double angleDelta){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //through exploiting the fact that angles are cyclic, just add the angle delta to current one
        this.poseLookup.add(new PFPose2d(prev.pos, prev.heading + angleDelta));
        return this;
    }

    public PFPath turnTo(double targetAngle){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //through exploiting the fact that angles are cyclic, just set the heading to the target
        this.poseLookup.add(new PFPose2d(prev.pos, targetAngle));
        return this;
    }

    public PFPath move(double distX, double distY){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //combine advance and strafe (relative) into one move
        this.poseLookup.add(new PFPose2d(prev.pos.plus(prev.getNormal().times(distX)).plus(prev.getDir().times(distY)), prev.heading));
        return this;
    }

    public PFPath moveTo(Vector2d globalPos){
        PFPose2d prev = this.poseLookup.get(poseLookup.size() - 1);
        //just set the new pose's position to the one in the argument
        this.poseLookup.add(new PFPose2d(globalPos, prev.heading));
        return this;
    }
}
