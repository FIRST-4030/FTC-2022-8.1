package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.velocityramping;

public class VelocityRamping {

    public double MAX_VELOCITY;
    public double acceleration;

    /**
     * You must input a max velocity (in meters per second) in order to use it
     * @param maxVel
     */
    public VelocityRamping(double maxVel){
        this.MAX_VELOCITY = maxVel;
    }

    /**
     * The solve function takes in two inputs: distance (m/s) and expected time (seconds)
     * @param distance
     * @param time
     */
    public VelocityRamping solve(double distance, double time){
        double distMax = Math.min((2 * distance) / time, MAX_VELOCITY);
        acceleration = distMax / (time - (distance / distMax));
        return this;
    }
}
