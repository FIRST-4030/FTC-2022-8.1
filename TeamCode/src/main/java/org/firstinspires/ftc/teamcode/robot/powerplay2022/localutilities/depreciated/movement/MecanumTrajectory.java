package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULArrays;

import java.util.Stack;

public class MecanumTrajectory {

    public Stack<Pair<String, Double>> cmdStack;

    public MecanumTrajectory(){
        cmdStack = new Stack<>();
    }

    public Stack<Pair<String, Double>> getCmdStack(){
        return cmdStack;
    }

    public MecanumTrajectory forward(double t){
        cmdStack.add(new Pair<>("forward", t));
        return this;
    }
    public MecanumTrajectory back(double t){
        cmdStack.add(new Pair<>("back", t));
        return this;
    }
    public MecanumTrajectory left(double t){
        cmdStack.add(new Pair<>("left", t));
        return this;
    }
    public MecanumTrajectory right(double t){
        cmdStack.add(new Pair<>("right", t));
        return this;
    }
    public MecanumTrajectory turnLeft(double t){
        cmdStack.add(new Pair<>("turnLeft", t));
        return this;
    }
    public MecanumTrajectory turnRight(double t){
        cmdStack.add(new Pair<>("turnRight", t));
        return this;
    }
    public MecanumTrajectory idle(double t){
        cmdStack.add(new Pair<>("idle", t));
        return this;
    }

    public void build(){
        cmdStack = EULArrays.stackFlip(cmdStack);
    }
}
