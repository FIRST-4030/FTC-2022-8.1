package org.firstinspires.ftc.teamcode.utils.sensors.pot;

import org.firstinspires.ftc.teamcode.utils.general.misc.Available;

public interface Potentiometer extends Available {

    //millivolts
    public double getMV();

    //degrees
    public double getAngleD();

    //radians
    public double getAngleR();

    //is it zero
    public boolean isZero();
}
