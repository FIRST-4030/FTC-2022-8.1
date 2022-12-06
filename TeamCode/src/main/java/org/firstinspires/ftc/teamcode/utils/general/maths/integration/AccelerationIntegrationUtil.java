package org.firstinspires.ftc.teamcode.utils.general.maths.integration;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;

public interface AccelerationIntegrationUtil extends BNO055IMU.AccelerationIntegrator {
    Position getLastPosition();
    Velocity getLastVelocity();
    Acceleration getLastAcceleration();
    Acceleration getAccelerationDerivative();

}
