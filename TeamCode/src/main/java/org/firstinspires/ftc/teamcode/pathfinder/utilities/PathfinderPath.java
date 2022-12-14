package org.firstinspires.ftc.teamcode.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

import java.util.ArrayList;
//TODO: Add documentation & more features
public class PathfinderPath {
    public ArrayList<PathfinderPose> poseHistory;

    public PathfinderPath(PathfinderPose initialPose){
        this.poseHistory = new ArrayList<>();
        this.poseHistory.add(initialPose);
    }

    public PathfinderPath advance(double measurement){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        poseToAdd.pos.plus(poseToAdd.dir.times(measurement));
        poseHistory.add(poseToAdd);
        return this;
    }

    public PathfinderPath strafe(double measurement){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        poseToAdd.pos.plus(poseToAdd.normal.times(measurement));
        poseHistory.add(poseToAdd);
        return this;
    }

    public PathfinderPath turn(double radians){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        Matrix2d rot = Matrix2d.makeRotation(radians);
        poseToAdd.dir = rot.times(poseToAdd.dir);
        poseToAdd.normal = new Vector2d(+poseToAdd.dir.y, -poseToAdd.dir.x);
        poseHistory.add(poseToAdd);
        return this;
    }

    public PathfinderPath turnTo(double radians){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        poseHistory.add(new PathfinderPose(poseToAdd.pos, radians));
        return this;
    }

    public PathfinderPath translate(Vector2d nPos){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        poseToAdd.pos.plus(poseToAdd.normal.times(nPos.x));
        poseToAdd.pos.plus(poseToAdd.dir.times(nPos.y));
        poseHistory.add(poseToAdd);
        return this;
    }

    public PathfinderPath translateTo(Vector2d nPos){
        PathfinderPose poseToAdd = poseHistory.get(poseHistory.size() - 1).copy();
        poseHistory.add(new PathfinderPose(nPos, poseToAdd.dir, poseToAdd.normal));
        return this;
    }
}
