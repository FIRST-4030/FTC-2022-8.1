package org.firstinspires.ftc.teamcode.utils.sensors.led_matrix.driver;

public enum BLINKING_MODE {
    OFF(0b0000),
    TWO_HZ(0b0010),
    ONE_HZ(0b0100),
    HALF_HZ(0b0110);

    private int mode;

    BLINKING_MODE(int mode) {
        this.mode = mode;
    }

    public int getMode() {
        return mode;
    }
}
