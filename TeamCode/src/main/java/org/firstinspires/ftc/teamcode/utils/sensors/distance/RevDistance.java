package org.firstinspires.ftc.teamcode.utils.sensors.distance;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This class implements a REV-31-1505 distance sensor from REV Robotics.
// A new class will need to be created for any other distance sensor.
// To make a new distance sensor, copy this class and change the constants
// in the bottom few methods that determine minDistance, maxDistance and field of view

public class RevDistance implements Distance {
    private DistanceSensor distanceSensor;

    public RevDistance(HardwareMap map, Telemetry telemetry, String name) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }
        try {
            distanceSensor = map.get(DistanceSensor.class, name);

        } catch (Exception e) {
            telemetry.log().add(this.getClass().getSimpleName() + ": No such device: " + name);
            return;
        }
    }

    @Override
    public boolean isAvailable() {
        return (distanceSensor != null);
    }

    public double distance(DistanceUnit units) {
        double measured = DistanceUnit.infinity;
        if (isAvailable()) {
            measured = distanceSensor.getDistance(units);
        }
        return measured;
    }

    public double distance() {
        return distance(DistanceUnit.MM);
    }

    @Override
    public double minDistance() {
        return 0.0;     // millimeters
    }

    @Override
    public double maxDistance() {
        return 2000.0;  // millimeters
    }

    @Override
    public double fieldOfView() {
        return 25.0;    // degrees
    }
}
