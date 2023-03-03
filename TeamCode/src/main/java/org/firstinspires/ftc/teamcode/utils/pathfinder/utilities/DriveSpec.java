package org.firstinspires.ftc.teamcode.utils.pathfinder.utilities;

public class DriveSpec {

    public double advancementTickPerMeasurementUnit, lateralTickPerMeasurementUnit, ticksPerTurn;
    public double trackWidth, wheelbase, expectedTurnRadius;

    public DriveSpec(double trackWidth, double wheelbase, double advancementTickPerMeasurementUnit, double lateralTickPerMeasurementUnit, double ticksPerTurn){
        this.advancementTickPerMeasurementUnit = advancementTickPerMeasurementUnit;
        this.lateralTickPerMeasurementUnit = lateralTickPerMeasurementUnit;
        this.ticksPerTurn = ticksPerTurn;

        this.trackWidth = trackWidth;
        this.wheelbase = wheelbase;
        this.expectedTurnRadius = Math.sqrt((wheelbase / 2) * (wheelbase / 2) + (trackWidth / 2) * (trackWidth / 2));
    }
}
