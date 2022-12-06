package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class PFPose2d{

    public Vector2d pos;
    public double heading; //store as angles

    public PFPose2d(Vector2d pos, double heading){
        this.pos = pos;
        this.heading = heading;
    }

    public PFPose2d(){
        this(new Vector2d(), 0);
    }

    public PFPose2d(Vector2d pos, Vector2d dir){
        this(pos, Math.copySign(Math.acos(dir.times(new Vector2d(0, 1))), dir.x));
    }

    public Vector2d getDir(){
        return new Vector2d(Math.sin(heading), Math.cos(heading));
    }

    public Vector2d getNormal(){
        Vector2d output = getDir();
        return new Vector2d(output.y, -output.x);
    }

    public Vector2d toRelativeAxis(Vector2d in){
        Vector2d yAxis = getDir(), xAxis = getNormal();
        Vector2d relativePos = in.minus(pos);
        return new Vector2d(xAxis.times(relativePos), yAxis.times(relativePos));
    }
}
