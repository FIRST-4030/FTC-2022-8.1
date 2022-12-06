package org.firstinspires.ftc.teamcode.utils.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.general.misc.Available;

public class Motor implements Available {
    public static final DcMotor.RunMode DEFAULT_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    protected Telemetry telemetry;
    protected DcMotorEx motor = null;
    private boolean enabled = true;
    protected int offset = 0;

    public Motor(HardwareMap map, Telemetry telemetry, MotorConfig config) {
        if (config == null) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Null config");
            return;
        }
        if (config.name == null || config.name.isEmpty()) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Null/empty name");
            return;
        }
        try {
            motor = (DcMotorEx)map.dcMotor.get(config.name);
            setReverse(config.reverse);
            setBrake(config.brake);
            setMode(config.mode);
            resetEncoder();
        } catch (Exception e) {
            telemetry.log().add(this.getClass().getSimpleName() + " No such device: " + config.name);
            motor = null;
        }
        this.telemetry = telemetry;
    }
    public boolean isBusy(){return motor.isBusy();}
    public void setVelocity(double v){
        motor.setVelocity(v);
    }

    public void setVelocity(double v, AngleUnit u){
        motor.setVelocity(v, u);
    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stop();
        }
    }

    public boolean isAvailable() {
        return enabled && (motor != null);
    }

    public void setPower(float power) {
        if (!isAvailable()) {
            return;
        }
        //if (isPositionPID()) {
        //    telemetry.log().add(this.getClass().getSimpleName() + ": Position PID active");
        //    return;
        //}
        motor.setPower(power);
    }

    public void stop() {
        if (isPositionPID()) {
            setMode(null);
        }
        setPower(0);
    }

    public int getEncoder() {
        if (!isAvailable()) {
            return 0;
        }
        return motor.getCurrentPosition() + offset;
    }
    public DcMotorEx get() {
        if (!isAvailable()) {
            return null;
        }
        return motor;
    }

    public void resetEncoder() {
        stop();
        DcMotor.RunMode mode = getMode();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(mode);
        offset = -getEncoder();
    }

    public boolean isPositionPID() {
        return (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTarget(int target) {
        /*if (!isAvailable()) {
            return;
        }
        if (!isPositionPID()) {
            stop();
            telemetry.log().add(this.getClass().getSimpleName() + ": Position PID not active");
            return;
        }*/
        motor.setTargetPosition(target);
    }

    public int getTarget() {
        if (!isAvailable()) {
            return 0;
        }
        if (!isPositionPID()) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Position PID not active");
            return 0;
        }
        return motor.getTargetPosition() + offset;
    }

    public boolean onTarget() {
        if (!isAvailable()) {
            return false;
        }
        if (!isPositionPID()) {
            telemetry.log().add(this.getClass().getSimpleName() + ": Position PID not active");
            return false;
        }
        return !motor.isBusy();
    }

    public void setMode(DcMotor.RunMode mode) {
        if (!isAvailable()) {
            return;
        }
        if (mode == null) {
            mode = DEFAULT_MODE;
        }
        motor.setMode(mode);
        if (isPositionPID()) {
            motor.setTargetPosition(motor.getCurrentPosition());
        }
    }

    public DcMotor.RunMode getMode() {
        DcMotor.RunMode mode;
        try {
            mode = motor.getMode();
        } catch (Exception e) {
            mode = DEFAULT_MODE;
        }
        return mode;
    }

    public void setBrake(boolean brake) {
        if (!isAvailable()) {
            return;
        }
        DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.FLOAT;
        if (brake) {
            behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        }
        motor.setZeroPowerBehavior(behavior);
    }

    public boolean getBrake() {
        if (!isAvailable()) {
            return false;
        }
        return (motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setReverse(boolean brake) {
        if (!isAvailable()) {
            return;
        }
        DcMotor.Direction direction = DcMotor.Direction.FORWARD;
        if (brake) {
            direction = DcMotor.Direction.REVERSE;
        }
        motor.setDirection(direction);
    }

    public boolean getReverse() {
        if (!isAvailable()) {
            return false;
        }
        return (motor.getDirection() == DcMotor.Direction.REVERSE);
    }
}
