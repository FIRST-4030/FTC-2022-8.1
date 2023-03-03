package org.firstinspires.ftc.teamcode.utils.pathfinder.utilities;

public abstract class MotionMaker {

    public abstract void buildPath(PathfinderPath path, EncoderTickBuffer[] encoderTickBuffers);

    public abstract void process(PathfinderPose currentPose, PathfinderPose targetPose, PathfinderPath.TransitionType type, EncoderTickBuffer[] rwEncoderBuffers);
}
