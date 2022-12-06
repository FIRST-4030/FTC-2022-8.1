package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class VectorAxis {

    public Vector2d xAxis, yAxis, pos;

    public VectorAxis(Vector2d vector, boolean isForward){
        this.pos = new Vector2d();
        if (isForward){
            this.yAxis = vector;
            this.xAxis = new Vector2d(this.yAxis.y, -this.yAxis.x);
        } else {
            this.xAxis = vector;
            this.yAxis = new Vector2d(-this.xAxis.y, this.xAxis.x);
        }
    }

    public VectorAxis(Vector2d pos, Vector2d vector, boolean isForward){
        this.pos = pos;
        if (isForward){
            this.yAxis = vector;
            this.xAxis = new Vector2d(this.yAxis.y, -this.yAxis.x);
        } else {
            this.xAxis = vector;
            this.yAxis = new Vector2d(-this.xAxis.y, this.xAxis.x);
        }
    }

    public Vector2d toRelative(Vector2d in){
        return new Vector2d(xAxis.times(in.minus(pos)), yAxis.times(in.minus(pos)));
    }
}
