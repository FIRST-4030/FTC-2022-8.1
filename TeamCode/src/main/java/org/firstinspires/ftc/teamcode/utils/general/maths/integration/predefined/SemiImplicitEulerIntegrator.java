package org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;

public class SemiImplicitEulerIntegrator implements ImuIntegration{

    private double lastElapsedTime, currentElapsedTime;

    private Vector3d currentAccel, currentVeloc, currentPos;
    private Vector3d lastAccel, lastVeloc, lastPos;

    @Override
    public double getLastElapsedTime() {
        return lastElapsedTime;
    }

    @Override
    public Vector3d getLastAccel() {
        return lastAccel;
    }

    @Override
    public Vector3d getLastVelocity() {
        return lastVeloc;
    }

    @Override
    public Vector3d getLastPosition() {
        return lastPos;
    }

    @Override
    public double getCurrentElapsedTime() {
        return currentElapsedTime;
    }

    @Override
    public Vector3d getCurrentAccel() {
        return currentAccel;
    }

    @Override
    public Vector3d getCurrentVelocity() {
        return currentVeloc;
    }

    @Override
    public Vector3d getCurrentPosition() {
        return currentPos;
    }

    @Override
    public void init() {
        lastElapsedTime = 0;
        lastAccel = new Vector3d();
        lastVeloc = new Vector3d();
        lastPos = new Vector3d();

        currentElapsedTime = 0;
        currentAccel = new Vector3d();
        currentVeloc = new Vector3d();
        currentPos = new Vector3d();
    }

    @Override
    public void integrate(Acceleration accel, double deltaTimeMs) {
        lastElapsedTime = currentElapsedTime;
        currentElapsedTime += deltaTimeMs;

        double deltaTime = deltaTimeMs * EULConstants.MS2SEC;

        lastAccel = currentAccel;
        currentAccel = new Vector3d(accel.xAccel, accel.yAccel, accel.zAccel);

        lastVeloc = currentVeloc;
        currentVeloc = currentVeloc.plus(currentAccel.times(deltaTime));

        lastPos = currentPos;
        currentPos = currentPos.plus(currentVeloc.times(deltaTime));
    }
}
