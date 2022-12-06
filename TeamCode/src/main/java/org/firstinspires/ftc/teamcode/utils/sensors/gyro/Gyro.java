package org.firstinspires.ftc.teamcode.utils.sensors.gyro;

import org.firstinspires.ftc.teamcode.utils.general.misc.Available;

public interface Gyro extends Available {
    boolean isReady();

    float getHeading();

    float getRaw();

    void setOffset(float offset);
}
