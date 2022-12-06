package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;

public class AngleConversion {

    public enum MODE{
        RADIANS,
        DEGREES,
        TURNS
    }

    public interface Equation{
        void setMode(AngleConversion.MODE mode);
        double solve(double input);
    }

    public static class Centered implements Equation{
        MODE mode;
        @Override
        public void setMode(MODE mode) {
            this.mode = mode;
        }

        @Override
        public double solve(double input) {
            double output = 0.5;
            switch (mode){
                case DEGREES:
                    output += 0.5 * (input / 135);
                    break;
                case TURNS:
                    output += 0.5 * (input / 0.375);
                    break;
                default:
                    output += 0.5 * (input / (3 * Math.PI / 4));
            }
            return output;
        }
    }

    public static class CenteredReverse implements Equation{
        MODE mode;
        @Override
        public void setMode(MODE mode) {
            this.mode = mode;
        }

        @Override
        public double solve(double input) {
            double output = 0.5;
            switch (mode){
                case DEGREES:
                    output += 0.5 * ((input - 180) / 135);
                    break;
                case TURNS:
                    output += 0.5 * ((input - 0.5) / 0.375);
                    break;
                default:
                    output += 0.5 * ((input - Math.PI) / (3 * Math.PI / 4));
            }
            return output;
        }
    }

    public Equation equation;
    public static final double SERVO_MAX = 1.0, SERVO_MIN = 0.0;

    public AngleConversion(Equation conversion, MODE mode){
        this.equation = conversion;
        this.equation.setMode(mode);
    }

    public double angleToServo(double input){
        return EULMathEx.doubleClamp(SERVO_MIN, SERVO_MAX, equation.solve(input));
    }
}
