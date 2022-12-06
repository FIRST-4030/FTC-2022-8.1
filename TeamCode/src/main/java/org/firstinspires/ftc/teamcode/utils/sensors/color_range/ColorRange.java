package org.firstinspires.ftc.teamcode.utils.sensors.color_range;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.general.misc.Available;

public interface ColorRange extends Available {

    void enableLight(boolean enable);

    double distance();

    NormalizedRGBA color();
}