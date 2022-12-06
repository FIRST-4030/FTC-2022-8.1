package org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.accelintegration;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.AccelerationIntegrationUtil;

public class AccelIntegratorRK4 implements AccelerationIntegrationUtil {

    private static class State{
        public Position pos;
        public Velocity vel;

        public State(){
            this.pos = new Position();
            this.vel = new Velocity();
        }

        public State(Position pos, Velocity vel){
            this.pos = pos;
            this.vel = vel;
        }
    }

    private static class Derivative{
        public Velocity dPos;
        public Acceleration dVel;

        public Derivative(){
            this.dPos = new Velocity();
            this.dVel = new Acceleration();
        }

        public Derivative(Velocity dPos, Acceleration dVel){
            this.dPos = dPos;
            this.dVel = dVel;
        }
    }

    private Position lastPosition;
    private Velocity lastVelocity;
    private Acceleration lastAcceleration;

    private Position currentPosition;
    private Velocity currentVelocity;
    private Acceleration currentAcceleration;

    private State lastState;
    private Derivative lastDerivative;

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
        output.xAccel = (currentAcceleration.xAccel - lastAcceleration.xAccel) / ((currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime) * EULConstants.NANO2SEC);
        output.yAccel = (currentAcceleration.yAccel - lastAcceleration.yAccel) / ((currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime) * EULConstants.NANO2SEC);
        output.zAccel = (currentAcceleration.zAccel - lastAcceleration.zAccel) / ((currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime) * EULConstants.NANO2SEC);
        output.acquisitionTime = currentAcceleration.acquisitionTime - lastAcceleration.acquisitionTime;
        return output;
    }


    private Derivative calcAcceleration(State initial, double dt, double in, Derivative derivative){

        State state = initial;
        state.pos.x += derivative.dPos.xVeloc * dt;
        state.pos.y += derivative.dPos.yVeloc * dt;
        state.pos.z += derivative.dPos.zVeloc * dt;
        state.vel.xVeloc += derivative.dVel.xAccel * dt;
        state.vel.yVeloc += derivative.dVel.yAccel * dt;
        state.vel.zVeloc += derivative.dVel.zAccel * dt;

        Acceleration accelDir = getAccelerationDerivative();

        Derivative output = new Derivative();
        output.dPos = state.vel;
        output.dVel.xAccel = currentAcceleration.xAccel + accelDir.xAccel * in;
        output.dVel.yAccel = currentAcceleration.yAccel + accelDir.yAccel * in;
        output.dVel.zAccel = currentAcceleration.zAccel + accelDir.zAccel * in;
        output.dVel.acquisitionTime = (long) in;

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
        Derivative a, b, c, d, output = new Derivative();
        lastAcceleration = currentAcceleration;
        currentAcceleration = linearAcceleration;

        double dt = getAccelerationDerivative().acquisitionTime * EULConstants.NANO2SEC;

        a = calcAcceleration(lastState,      dt,          dt, lastDerivative);
        b = calcAcceleration(lastState,      dt, dt*0.5f,              a);
        c = calcAcceleration(lastState,      dt, dt*0.5f,              b);
        d = calcAcceleration(lastState,      dt,          dt,              c);

        output.dPos.xVeloc = 1/6d * (a.dPos.xVeloc + 2 * (b.dPos.xVeloc + c.dPos.xVeloc) + d.dPos.xVeloc);
        output.dPos.yVeloc = 1/6d * (a.dPos.yVeloc + 2 * (b.dPos.yVeloc + c.dPos.yVeloc) + d.dPos.yVeloc);
        output.dPos.zVeloc = 1/6d * (a.dPos.zVeloc + 2 * (b.dPos.zVeloc + c.dPos.zVeloc) + d.dPos.zVeloc);

        output.dVel.xAccel = 1/6d * (a.dVel.xAccel + 2 * (b.dVel.xAccel + c.dVel.xAccel) + d.dVel.xAccel);
        output.dVel.yAccel = 1/6d * (a.dVel.yAccel + 2 * (b.dVel.yAccel + c.dVel.yAccel) + d.dVel.yAccel);
        output.dVel.zAccel = 1/6d * (a.dVel.zAccel + 2 * (b.dVel.zAccel + c.dVel.zAccel) + d.dVel.zAccel);

        //calculate the new positions and velocity using newly calculated derivative from the RK4 main core
        lastState.pos.x += output.dPos.xVeloc * dt;
        lastState.pos.y += output.dPos.yVeloc * dt;
        lastState.pos.z += output.dPos.zVeloc * dt;

        lastState.vel.xVeloc += output.dVel.xAccel * dt;
        lastState.vel.yVeloc += output.dVel.yAccel * dt;
        lastState.vel.zVeloc += output.dVel.zAccel * dt;

        //swap variables
        lastDerivative = output;

        lastPosition = currentPosition;
        currentPosition = lastState.pos;

        lastVelocity = currentVelocity;
        currentVelocity = lastState.vel;
    }


}
