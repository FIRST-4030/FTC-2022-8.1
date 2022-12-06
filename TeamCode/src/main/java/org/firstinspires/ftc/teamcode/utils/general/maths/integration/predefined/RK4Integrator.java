package org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.accelintegration.AccelIntegratorRK4;

public class RK4Integrator implements ImuIntegration{

    private static class State{
        public Vector3d pos;
        public Vector3d vel;

        public State(){
            this.pos = new Vector3d();
            this.vel = new Vector3d();
        }

        public State(Vector3d pos, Vector3d vel){
            this.pos = pos;
            this.vel = vel;
        }
    }

    private static class Derivative{
        public Vector3d dPos;
        public Vector3d dVel;

        public Derivative(){
            this.dPos = new Vector3d();
            this.dVel = new Vector3d();
        }

        public Derivative(Vector3d dPos, Vector3d dVel){
            this.dPos = dPos;
            this.dVel = dVel;
        }
    }

    private double lastElapsedTime, currentElapsedTime;

    private Vector3d currentAccel;
    private Vector3d lastAccel;
    private State lastState, currentState;
    private Derivative lastDerivative, currentDerivative;

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
        return lastState.vel;
    }

    @Override
    public Vector3d getLastPosition() {
        return lastState.pos;
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
        return currentState.vel;
    }

    @Override
    public Vector3d getCurrentPosition() {
        return currentState.pos;
    }

    @Override
    public void init() {
        lastElapsedTime = 0;
        lastAccel = new Vector3d();
        lastState = new State();
        lastDerivative = new Derivative();

        currentElapsedTime = 0;
        currentAccel = new Vector3d();
        currentState = new State();
        currentDerivative = new Derivative();

    }

    private Derivative calcDerivative(State initial, double dt1, double dt2, Derivative derivative){
        State state = initial;
        state.pos.x += derivative.dPos.x * dt1;
        state.pos.y += derivative.dPos.y * dt1;
        state.pos.z += derivative.dPos.z * dt1;
        state.vel.x += derivative.dVel.x * dt1;
        state.vel.y += derivative.dVel.y * dt1;
        state.vel.z += derivative.dVel.z * dt1;

        Vector3d accelSlope = (currentAccel.minus(lastAccel).div((currentElapsedTime - lastElapsedTime))).times(EULConstants.MS2SEC);

        Derivative output = new Derivative();
        output.dPos = state.vel;
        output.dVel.x = currentAccel.x + accelSlope.x * dt2;
        output.dVel.y = currentAccel.y + accelSlope.y * dt2;
        output.dVel.z = currentAccel.z + accelSlope.z * dt2;

        return output;
    }

    @Override
    public void integrate(Acceleration accel, double deltaTimeMs) {
        Derivative a, b, c, d, output = new Derivative();
        lastElapsedTime = currentElapsedTime;
        currentElapsedTime += deltaTimeMs;

        double deltaTime = deltaTimeMs * EULConstants.MS2SEC;
        lastAccel = currentAccel;
        currentAccel = new Vector3d(accel.xAccel, accel.yAccel, accel.zAccel);

        lastState = currentState;

        lastDerivative = currentDerivative;

        a = calcDerivative(lastState, deltaTime, deltaTime * 0.5, new Derivative());
        b = calcDerivative(lastState, deltaTime, deltaTime * 0.5,                a);
        c = calcDerivative(lastState, deltaTime, deltaTime * 0.5,                b);
        d = calcDerivative(lastState, deltaTime, deltaTime * 1  ,                c);

        output.dPos.x = 1/6d * (a.dPos.x + 2 * (b.dPos.x + c.dPos.x) + d.dPos.x);
        output.dPos.y = 1/6d * (a.dPos.y + 2 * (b.dPos.y + c.dPos.y) + d.dPos.y);
        output.dPos.z = 1/6d * (a.dPos.z + 2 * (b.dPos.z + c.dPos.z) + d.dPos.z);

        output.dVel.x = 1/6d * (a.dVel.x + 2 * (b.dVel.x + c.dVel.x) + d.dVel.x);
        output.dVel.y = 1/6d * (a.dVel.y + 2 * (b.dVel.y + c.dVel.y) + d.dVel.y);
        output.dVel.z = 1/6d * (a.dVel.z + 2 * (b.dVel.z + c.dVel.z) + d.dVel.z);

        currentState.pos.x += output.dPos.x * deltaTime;
        currentState.pos.y += output.dPos.y * deltaTime;
        currentState.pos.z += output.dPos.z * deltaTime;

        currentState.vel.x += output.dVel.x * deltaTime;
        currentState.vel.y += output.dVel.y * deltaTime;
        currentState.vel.z += output.dVel.z * deltaTime;

        currentDerivative = output;
    }
}
