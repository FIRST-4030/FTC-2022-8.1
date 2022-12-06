package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.customDriver;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULArrays;

import java.util.Stack;

public class CustomServoDriver {

    public static int precision = 4;
    public static boolean initialized = false;
    private static double multiplier = 10000;

    private abstract class equation{

        protected double a, b;
        abstract double solve(double t);
    }

    private class lerp extends equation{

        private lerp(double lim1, double lim2){
            this.a = lim1;
            this.b = lim2;
        }


        @Override
        public double solve(double t) {
            return a + (b - a) * t;
        }
    }

    private class slerp extends equation{

        private slerp(Vector2d first, Vector2d second){
            a = 0;
            b = EULMathEx.safeACOS(first.times(second));
        }

        @Override
        public double solve(double t) {
            return a + (b - a) * t;
        }
    }

    public enum TYPE{
        DS180,
        DS270,
        DS360
    }

    public enum METHOD{
        LERP,
        SLERP
    }

    public static final CustomServoDriver SERVO180 = new CustomServoDriver(TYPE.DS180);
    public static final CustomServoDriver SERVO270 = new CustomServoDriver(TYPE.DS270);
    public static final CustomServoDriver SERVO360 = new CustomServoDriver(TYPE.DS360);

    private TYPE type;

    public CustomServoDriver(TYPE type){
        this.type = type;
    }

    public TYPE getType(){
        return type;
    }

    public static void init(){
        CustomServoDriver.multiplier = 1;
        for (int i = 0; i < CustomServoDriver.precision; i++) {
            multiplier *= 10;
        }

        initialized = true;
    }

    public Stack<Double> generateServoPath(double startAngle, double endAngle, METHOD method){

        Stack<Double> output = new Stack<>();

        equation eq = null;

        switch (method){
            case LERP:
                eq = new lerp(startAngle, endAngle);
                break;
            case SLERP:
                Vector2d first = new Vector2d(1, 0);
                Vector2d second = (Matrix2d.makeRotation(endAngle - startAngle)).times(first);
                eq = new slerp(first, second);
                break;
        }

        double delta = eq.b - eq.a;
        int checkpoints = (int) Math.round(delta / (Math.PI / 3));
        //double sectionLengths = delta / checkpoints;

        output.push(angleToScalar(startAngle));
        for (int i = 1; i <= (checkpoints); i++) {
            output.push(eq.solve(1d / (checkpoints+1) * i));
        }
        output.push(angleToScalar(endAngle));

        return EULArrays.stackFlip(output);
    }

    public static double followServoPath(double currentPos, Stack<Double> positions){
        double output = currentPos;
        if (!positions.empty()){
            if (positions.peek() <= currentPos) positions.pop();
            output = positions.peek();
        }

        return roundPosition(output);
    }

    public static double followServoPath(double currentPos, double target, Stack<Double> positions){
        double output = currentPos;
        Stack<Double> usedPositions = positions;
        boolean ahead = target < currentPos;
        if(ahead) usedPositions = EULArrays.stackFlip(usedPositions);
        if(!usedPositions.isEmpty()) {
            while ((usedPositions.peek() <= currentPos && !ahead) || (usedPositions.peek() >= currentPos && ahead)) {
                usedPositions.pop();
            }
            if((target <= usedPositions.peek() && !ahead) || (target >= usedPositions.peek() && ahead)){
                output = target;
            }else{ output = usedPositions.peek(); }
        }

        return output;
    }

    public static void copyPositions(Stack<Double> target, Stack<Double> destination){
        Stack<Double> flipped = target;
        for (double n: flipped) {
            destination.push(n);
        }
    }

    public double angleToScalar(double in){
        double output = 0.5;
        switch (type){
            case DS180:
                output = (in % (Math.PI * 2)) / Math.PI;
                break;
            case DS270:
                output = (in % (Math.PI * 2)) / (3 * Math.PI / 2);
                break;
            case DS360:
                output = (in % (Math.PI * 2)) / (2 * Math.PI);
                break;
        }
        return output;
    }

    public double scalarToAngle(double in){
        double output = 0;
        switch (type){
            case DS180:
                output = in * Math.PI;
                break;
            case DS270:
                output = in * (3 * Math.PI / 2);
                break;
            case DS360:
                output = in * (2 * Math.PI);
                break;
        }
        return output;
    }

    public static double roundPosition(double in){
        return Math.round(in * multiplier) / multiplier;
    }
}
