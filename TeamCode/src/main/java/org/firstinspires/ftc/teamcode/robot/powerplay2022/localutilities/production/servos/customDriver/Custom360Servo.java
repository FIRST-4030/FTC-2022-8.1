package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.customDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.general.misc.VirtualGearbox;

import java.util.Vector;

public class Custom360Servo {

    private Servo servo;
    private VirtualGearbox gearing;
    private ServoCheckpoints currentCheckpoints;
    private double currentAngle;
    private static Vector<Custom360Servo> servos = new Vector<>();

    public Custom360Servo(HardwareMap hardwareMap, Telemetry telemetry, ServoConfig config, int gearIn, int gearOut){
        this.gearing = new VirtualGearbox(gearIn, gearOut);
        this.currentCheckpoints = null;
        this.currentAngle = 0;

        if (config == null) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Null config");
            return;
        }
        if (config.name == null || config.name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null/empty name");
        }

        try {
            this.servo = hardwareMap.servo.get(config.name);
            this.servo.setDirection(config.reverse ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        } catch (Exception e){
            servo = null;
            telemetry.log().add(this.getClass().getSimpleName() + "No such device: " + config.name);
        }

        servos.add(this);
    }

    public void dispose(){
        while(!currentCheckpoints.isDone) {
            setAngle(0);
        }
    }

    public static void disposeAll(){
        for (Custom360Servo servo: servos) {
            servo.dispose();
        }
    }

    public void setAngle(double angle){
        if (!(currentCheckpoints == null)){
            currentAngle = currentCheckpoints.getCheckpoint();
            ServoCheckpoints checkpoints = new ServoCheckpoints();
            if (!(checkpoints.makePath(currentAngle, angle * gearing.getInverseRatio()).naiveCheck(currentCheckpoints))){
                currentCheckpoints = checkpoints;
            } else if(!currentCheckpoints.isDone){
                currentCheckpoints.followPoints(servo.getPosition());
            }
        }
    }
}

class ServoCheckpoints{
    public double beginAngle, endAngle, delta;
    private float stableAAngle, stableBAngle;
    public int currentIdx;
    public Vector<Double> positions;
    public boolean isDone = true;


    public ServoCheckpoints(){
        this.beginAngle = 0;
        this.endAngle = 0;
        this.currentIdx = 0;
        this.positions = new Vector<>();
    }

    public double followPoints(double currentPosition){
        if ((currentPosition >= (positions.get(currentIdx) - 0.001)) && (currentPosition <= (positions.get(currentIdx) + 0.001))){
            incr();
        }

        if (currentIdx + 1 == positions.size()){
            isDone = true;
        } else {
            isDone = false;
        }

        return positions.get(currentIdx);
    }

    public double getBeginAngle(){
        return beginAngle;
    }

    public double getEndAngle(){
        return endAngle;
    }

    public int getCurrentIdx(){
        return currentIdx;
    }

    public void incr(){
        currentIdx += (currentIdx + 1) >= (positions.size()) ? 0 : 1;
    }

    public void decr(){
        currentIdx -= (currentIdx - 1) < 0 ? 0 : 1;
    }

    public ServoCheckpoints makePath(double currentAngle, double targetAngle){
        delta = targetAngle - currentAngle;
        double rotations = Math.floor(delta / (EULConstants.TAU * 0.45)) - 1;
        double savedPosition = (currentAngle / EULConstants.TAU) % 1;
        fillAngles(targetAngle);
        isDone = false;
        positions.clear();

        for (int i = 0; i < rotations; i++) {
            positions.add(savedPosition);
            savedPosition += 0.45;
            savedPosition %= 1;
        }

        positions.add((targetAngle / EULConstants.TAU) % 1);

        return this;
    }

    protected void fillAngles(double endAngle){
        this.endAngle = endAngle;
        this.stableBAngle = trunc4(endAngle);
    }

    protected float trunc4(double in){
        return (float) (Math.floor(in * 10000) / 10000);
    }

    public boolean naiveCheck(ServoCheckpoints checkpoints){
        return (checkpoints.stableBAngle == this.stableBAngle);
    }

    public double getCheckpoint(){
        return this.beginAngle + this.delta * getCurrentIdx();
    }
}
