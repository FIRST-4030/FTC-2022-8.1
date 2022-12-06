package org.firstinspires.ftc.teamcode.utils.sensors.distance;

import org.firstinspires.ftc.teamcode.utils.general.misc.Available;

public interface Distance extends Available {

    double minDistance();
    double maxDistance();
    double fieldOfView();
    double distance();
}
