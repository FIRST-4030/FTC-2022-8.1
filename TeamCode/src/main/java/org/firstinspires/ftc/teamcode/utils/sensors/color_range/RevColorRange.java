package org.firstinspires.ftc.teamcode.utils.sensors.color_range;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevColorRange implements ColorRange {
    public NormalizedColorSensor color;
    public DistanceSensor distance;

    public RevColorRange(HardwareMap map, Telemetry telemetry, String name) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }
        try {
            color = map.get(NormalizedColorSensor.class, name);
            distance = map.get(DistanceSensor.class, name);

        } catch (Exception e) {
            color = null;
            telemetry.log().add(this.getClass().getSimpleName() + ": No such device: " + name);
            return;
        }
    }

    public void enableLight(boolean enable) {
        if (!isAvailable()) {
            return;
        }
        if (!(color instanceof SwitchableLight)) {
            return;
        }
        ((SwitchableLight) color).enableLight(enable);
    }

    public boolean isAvailable() {
        return (color != null);
    }

    public NormalizedRGBA color() {
        NormalizedRGBA colors = new NormalizedRGBA();
        if (isAvailable()) {
            colors = color.getNormalizedColors();
        }
        return colors;
    }

    public double distance() {
        double mm = 0;
        if (isAvailable()) {
            mm = distance.getDistance(DistanceUnit.MM);
        }
        return mm;
    }

    public int get() {
        return (int) distance();
    }
}
