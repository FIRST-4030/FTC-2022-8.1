package org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.accelintegration;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.AccelerationIntegrationUtil;

public class AccelIntegratorSemiImplicitEuler implements AccelerationIntegrationUtil {

    private Position lastPosition;
    private Velocity lastVelocity;
    private Acceleration lastAcceleration;

    private Position currentPosition;
    private Velocity currentVelocity;
    private Acceleration currentAcceleration;

    @Override
    public Position getLastPosition() {
        return lastPosition;
    }

    @Override
    public Velocity getLastVelocity() {
        return lastVelocity;
    }

    @Override
    public Acceleration getLastAcceleration() {
        return lastAcceleration;
    }

    @Override
    public Acceleration getAccelerationDerivative() {
        Acceleration output = new Acceleration();
        output.xAccel = (currentAcceleration.xAccel - lastAcceleration.xAccel) / (currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime);
        output.yAccel = (currentAcceleration.yAccel - lastAcceleration.yAccel) / (currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime);
        output.zAccel = (currentAcceleration.zAccel - lastAcceleration.zAccel) / (currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime);
        output.acquisitionTime = currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime;
        return output;
    }

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        lastPosition = initialPosition;
        lastVelocity = initialVelocity;

        currentPosition = initialPosition;
        currentVelocity = initialVelocity;

        //manually hardcode the current acceleration to be 0 on all axes
        currentAcceleration = new Acceleration();
        currentAcceleration.xAccel = 0;
        currentAcceleration.yAccel = 0;
        currentAcceleration.zAccel = 0;
    }

    @Override
    public Position getPosition() {
        return currentPosition;
    }

    @Override
    public Velocity getVelocity() {
        return currentVelocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return currentAcceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        double deltaTime = linearAcceleration.acquisitionTime - currentAcceleration.acquisitionTime * EULConstants.NANO2SEC;

        lastAcceleration = currentAcceleration;
        currentAcceleration = linearAcceleration;

        //velocity += acceleration * deltaTime
        lastVelocity = currentVelocity;
        currentVelocity.xVeloc += currentAcceleration.xAccel * deltaTime;
        currentVelocity.yVeloc += currentAcceleration.yAccel * deltaTime;
        currentVelocity.zVeloc += currentAcceleration.zAccel * deltaTime;

        //position += velocity * deltaTime
        lastPosition = currentPosition;
        currentPosition.x += currentVelocity.xVeloc * deltaTime;
        currentPosition.y += currentVelocity.yVeloc * deltaTime;
        currentPosition.z += currentVelocity.zVeloc * deltaTime;
    }
}
