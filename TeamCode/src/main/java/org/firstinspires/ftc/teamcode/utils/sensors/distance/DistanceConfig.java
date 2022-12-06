package org.firstinspires.ftc.teamcode.utils.sensors.distance;

public class DistanceConfig {
    public final String name;
    public final DISTANCE_TYPES type;

    public DistanceConfig(DISTANCE_TYPES type, String name) {
        this.name = name;
        this.type = type;
    }
}
