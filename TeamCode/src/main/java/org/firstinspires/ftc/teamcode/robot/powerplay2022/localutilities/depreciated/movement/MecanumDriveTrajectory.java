package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.movement;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.utils.general.misc.VirtualRobot;

import java.util.Stack;

public class MecanumDriveTrajectory {

    private Stack<MecanumDriveState> cache;
    private Stack<MecanumDriveState> stateStack;
    private Stack<MecanumDriveState> copy;
    private VirtualRobot drive;
    AnglePID Apid;

    public MecanumDriveTrajectory(VirtualRobot drive){
        this.stateStack = new Stack<>();
        this.copy = new Stack<>();
        this.drive = drive;
        //Apid = new AnglePID(1/Math.PI, 0.000001, 1/4000);
        Apid = new AnglePID(1/Math.PI, 0, 1/4000);
    }

    public void build(){
        for (int i = 0; i < stateStack.size(); i++){
            this.copy.push(stateStack.get(i));
        }
    }

    public MecanumDriveState pop(){
        return copy.pop();
    }

    public MecanumDriveState peek(){
        return copy.peek();
    }

    public Stack<MecanumDriveState> getCurrentStateStack(){
        return copy;
    }

    public boolean isCurrentStateStackEmpty(){
        return copy.isEmpty();
    }

    public MecanumDriveTrajectory forward(double duration){
        cache.push(new MecanumDriveState("FORWARD", () -> {
            //drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = 1;
            //drive.virtualJoystick.z = 0;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory backward(double duration){
        cache.push(new MecanumDriveState("BACKWARDS", () -> {
            //drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = -1;
            //drive.virtualJoystick.z = 0;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory turnRight(double duration){
        cache.push(new MecanumDriveState("TURN_RIGHT", () -> {
            //drive.virtualJoystick.x = 0;
            //drive.virtualJoystick.y = 0;
            drive.virtualJoystick.z = 1;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory turnLeft(double duration){
        cache.push(new MecanumDriveState("TURN_LEFT", () -> {
            drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = 0;
            drive.virtualJoystick.z = -1;
        }, new MecanumDriveState.TimeCondition(duration)));
        return this;
    }

    public MecanumDriveTrajectory turnToAngle(double theta){
        Apid.update(drive.getDeltaTime(), theta, drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        cache.push(new MecanumDriveState("TURN_TO", () -> {
            drive.virtualJoystick.x = 0;
            drive.virtualJoystick.y = 0;
            drive.virtualJoystick.z = Apid.correctionPower;
        }, new MecanumDriveState.HeadingCondition(theta, 0.001)));
        return this;
    }

}
