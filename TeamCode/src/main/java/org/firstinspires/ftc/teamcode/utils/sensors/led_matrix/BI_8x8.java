package org.firstinspires.ftc.teamcode.utils.sensors.led_matrix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.sensors.led_matrix.driver.BLINKING_MODE;
import org.firstinspires.ftc.teamcode.utils.sensors.led_matrix.driver.HT16K33;

public class BI_8x8 implements LEDMatrix {
    public HT16K33 matrix;

    public BI_8x8(HardwareMap map, Telemetry telemetry, String name) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }
        try {
            matrix = map.get(HT16K33.class, name);

        } catch (Exception e) {
            telemetry.log().add(this.getClass().getSimpleName() + ": No such device: " + name);
            return;
        }
    }

    @Override
    public boolean isAvailable() {
        return (matrix != null);
    }

    // other things
    @Override
    public void setDisplayBuffer(byte[] displayBuffer) {
        if (displayBuffer.length != 16) return;

        matrix.displayBuffer = displayBuffer;
    }

    @Override
    public void reset() {
        matrix.clear();
        matrix.write();
        matrix.setBrightness(16);
        matrix.setBlinking(BLINKING_MODE.OFF);
    }

    // dragging this shit out of the driver class and into the interface
    @Override
    public void clear() {
        matrix.clear();
    }
    @Override
    public void write() {
        matrix.write();
    }
    @Override
    public void setBrightness(int brightness) {
        matrix.setBrightness(brightness);
    }
    @Override
    public void setBlinking(BLINKING_MODE mode) {
        matrix.setBlinking(mode);
    }
}
