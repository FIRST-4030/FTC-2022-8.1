package org.firstinspires.ftc.teamcode.utils.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.customDriver.CustomServoDriver;
import org.firstinspires.ftc.teamcode.utils.general.misc.Available;
import org.firstinspires.ftc.teamcode.utils.general.misc.RunOnce;

import java.util.Stack;

public class ServoFTC implements Available {

    public enum TYPE{
        D180,
        D270,
        D360
    }

    private class Gen implements Runnable{

        public TYPE type;

        public double startAngle = 0.00001 * 2*Math.PI, endAngle = 0.99999 * 2*Math.PI;

        public Stack<Double> output;

        public Gen(TYPE type){
            this.type = type;
        }

        @Override
        public void run() {
            switch (type){
                case D180:
                    output = CustomServoDriver.SERVO180.generateServoPath(startAngle, endAngle, CustomServoDriver.METHOD.LERP);
                    break;
                case D270:
                    output = CustomServoDriver.SERVO270.generateServoPath(startAngle, endAngle, CustomServoDriver.METHOD.LERP);

                    break;
                case D360:
                    output = CustomServoDriver.SERVO360.generateServoPath(startAngle, endAngle, CustomServoDriver.METHOD.LERP);

                    break;
            }
        }

        public double getStartAngle() {
            return startAngle;
        }

        public void setStartAngle(double startAngle) {
            this.startAngle = startAngle;
        }

        public double getEndAngle() {
            return endAngle;
        }

        public void setEndAngle(double endAngle) {
            this.endAngle = endAngle;
        }

        public Stack<Double> getOutput() {
            return output;
        }

        public void setOutput(Stack<Double> output) {
            this.output = output;
        }
    }

    private Servo servo;
    private static final double ABS_MIN = 0.0001d;
    private final static double ABS_MAX = 0.9999d;
    private double min = ABS_MIN;
    private double max = ABS_MAX;
    private Stack<Double> sta;
    private Gen pathGeneration;
    private RunOnce gen = new RunOnce() {
        @Override
        public void run() {
            pathGeneration.run();
        }
    };

    private boolean generation = false;

    public ServoFTC(HardwareMap map, Telemetry telemetry, ServoConfig config) {
        if (config == null) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Null config");
            return;
        }
        if (config.name == null || config.name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }
        try {
            servo = map.servo.get(config.name);
            if (config.reverse) {
                servo.setDirection(Servo.Direction.REVERSE);
            }
            this.min = config.min;
            this.max = config.max;
        } catch (Exception e) {
            servo = null;
            telemetry.log().add(this.getClass().getSimpleName() + "No such device: " + config.name);
        }


    }

    public void initalizeGeneration(TYPE type){
        pathGeneration = new Gen(type);
        generation = true;
    }

    public boolean isAvailable() {
        return servo != null;
    }

    public void setPosition(double position) {
        if (position < min) {
            position = min;
        } else if (position > max) {
            position = max;
        }
        setPositionRaw(position);
        /*
        if(Math.abs(getPosition() - position) < 0.5 || !generation){
            setPositionRaw(position);
        }else{
            gen.run();
            follow(pathGeneration.output, position);
        }
         */

    }

    public void setPositionRaw(double position) {
        if (!isAvailable()) {
            return;
        }
        if (position < ABS_MIN) {
            position = ABS_MIN;
        } else if (position > ABS_MAX) {
            position = ABS_MAX;
        }
        servo.setPosition(position);
    }

    public void follow(Stack<Double> stack, double target){
        servo.setPosition(CustomServoDriver.followServoPath(getPosition(), target ,stack));
    }

    public double getPosition() {
        if (!isAvailable()) {
            return ABS_MIN;
        }
        return servo.getPosition();
    }

    public void min() {
        setPositionRaw(min);
    }

    public void max() {
        setPositionRaw(max);
    }

    public void toggle() {
        double mid = (max + min) / 2;
        if (getPosition() >= mid) {
            min();
        } else {
            max();
        }
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }
}
