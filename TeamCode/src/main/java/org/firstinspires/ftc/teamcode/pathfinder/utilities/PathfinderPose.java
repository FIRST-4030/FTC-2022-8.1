package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
//TODO: Add documentation
public class PathfinderPose {

    public Vector2d pos, dir, normal;

    public PathfinderPose(Vector2d initialPos, double angle){
        this.pos = initialPos;
        this.dir = new Vector2d(Math.sin(angle), Math.cos(angle));
        this.normal = new Vector2d(+dir.y, -dir.x);
    }

    public PathfinderPose(){
        this(new Vector2d(), 0);
    }

    protected PathfinderPose(Vector2d pos, Vector2d dir, Vector2d normal){
        this.pos = pos;
        this.dir = dir;
        this.normal = normal;
    }

    public PathfinderPose copy(){
        return new PathfinderPose(pos, dir, normal);
    }

    public Vector2d toRelative(Vector2d globalCoord){
        Vector2d relativePos = globalCoord.minus(pos);
        return new Vector2d(relativePos.times(normal), relativePos.times(dir));
    }
}
