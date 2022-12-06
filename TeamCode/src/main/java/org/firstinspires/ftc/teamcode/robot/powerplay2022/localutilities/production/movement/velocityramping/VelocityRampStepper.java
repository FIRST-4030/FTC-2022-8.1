package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.velocityramping;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.VectorAxis;

import java.util.ArrayList;

public class VelocityRampStepper {
    
    public static class VelocityRampEquation{
        private double accel, deceleration, maxAccel;
        private double xInt, sign;

        public VelocityRampEquation(double maxAccel, double accel, double time, double sign){
            this.maxAccel = maxAccel;
            this.accel = accel;
            this.deceleration = -accel;
            this.xInt = time;
            this.sign = sign;
        }

        public double solve(double time){
            return Math.min(Math.min(accel * time, deceleration * (time - xInt)), maxAccel) * sign;
        }
    }
    
    public VelocityRamping accelForward, accelStrafe;
    
    private ArrayList<VelocityRampEquation[]> accelEquations;
    public double estimatedElapsedTime, stateElapsedTime;
    public int currentRamp;

    private VectorAxis absoluteAxis;
    
    public VelocityRampStepper(){
        this.accelForward = new VelocityRamping(1);
        this.accelStrafe = new VelocityRamping(1);
        init();
    }

    public VelocityRampStepper(VelocityRamping forwardRamping, VelocityRamping strafeRamping){
        this.accelForward = forwardRamping;
        this.accelStrafe = strafeRamping;
        init();
    }

    private void init(){
        accelEquations = new ArrayList<>(1);
        estimatedElapsedTime = 0;
        stateElapsedTime = 0;
        currentRamp = 0;
        absoluteAxis = new VectorAxis(new Vector2d(0, 1), true);
    }

    public void addRampForward(double sign, double distance, double time){
        estimatedElapsedTime += time;
        accelEquations.add(
                new VelocityRampEquation[]{
                        new VelocityRampEquation(
                            accelForward.MAX_VELOCITY,
                            accelForward.solve(distance, time).acceleration,
                            time,
                            sign),
                        new VelocityRampEquation(
                                accelStrafe.MAX_VELOCITY,
                                0,
                                time,
                                1)
                }
        );
    }

    public void addRampStrafe(double sign, double distance, double time){
        estimatedElapsedTime += time;
        accelEquations.add(
                new VelocityRampEquation[]{
                        new VelocityRampEquation(
                                accelForward.MAX_VELOCITY,
                                0,
                                time,
                                1),
                        new VelocityRampEquation(
                                accelStrafe.MAX_VELOCITY,
                                accelStrafe.solve(distance, time).acceleration,
                                time,
                                sign)
                }
        );
    }

    public void addRamp(Vector2d position, double time){
        estimatedElapsedTime += time;
        Vector2d coefficients = absoluteAxis.toRelative(position.normalized());
        double distance = position.length();
        VelocityRampEquation[] equations = new VelocityRampEquation[2];

        equations[0] = new VelocityRampEquation( // forward/backwards direction (in order of sign: 1, -1)
                accelForward.MAX_VELOCITY,
                accelForward.solve(coefficients.y, time).acceleration,
                time, Math.signum(coefficients.y));
        equations[1] = new VelocityRampEquation( // right/left direction (in order of sign: 1, -1)
                accelStrafe.MAX_VELOCITY,
                accelStrafe.solve(coefficients.x, time).acceleration,
                time, Math.signum(coefficients.x));

        accelEquations.add(equations);
    }

    public double[] update(double deltaTime){
        stateElapsedTime += deltaTime;
        if (stateElapsedTime > accelEquations.get(currentRamp)[0].xInt){
            currentRamp += 1; //increment current ramp index by 1
            stateElapsedTime = 0; //reset current ramp's time in state
        }
        return new double[]{
                accelEquations.get(currentRamp)[0].solve(stateElapsedTime) / accelForward.MAX_VELOCITY,
                accelEquations.get(currentRamp)[1].solve(stateElapsedTime) / accelStrafe.MAX_VELOCITY
        };
    }
}
