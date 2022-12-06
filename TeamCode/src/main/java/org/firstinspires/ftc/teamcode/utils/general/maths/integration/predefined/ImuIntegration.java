package org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;

public interface ImuIntegration {

    double getLastElapsedTime();
    Vector3d getLastAccel();
    Vector3d getLastVelocity();
    Vector3d getLastPosition();

    double getCurrentElapsedTime();
    Vector3d getCurrentAccel();
    Vector3d getCurrentVelocity();
    Vector3d getCurrentPosition();

    void init();
    void integrate(Acceleration accel, double deltaTimeMs);
}
