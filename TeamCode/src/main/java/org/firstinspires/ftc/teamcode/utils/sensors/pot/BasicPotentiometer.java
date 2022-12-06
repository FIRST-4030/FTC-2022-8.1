package org.firstinspires.ftc.teamcode.utils.sensors.pot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.general.maths.piecewise.PiecewiseInterpolation;
public class BasicPotentiometer implements Potentiometer {

    //hardware
    private AnalogInput pot;

    private final double scaleRad = Math.PI / 180;

    private final PiecewiseInterpolation poly;

    private Telemetry telemetry;

    //initalize hardware
    public BasicPotentiometer(HardwareMap map, Telemetry telemetry, String name, double[] constants, double[]    crackers){
        if(name == null || name.isEmpty()) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": invalid name - nonexistent");
        }
        this.telemetry = telemetry;
        try {
            pot = map.get(AnalogInput.class, name);
        } catch (Exception e) {
            telemetry.log().add(this.getClass().getSimpleName() + ": nope nope plug this in");
        }
        this.poly = new PiecewiseInterpolation(constants, crackers);
    }

    //see overridden methods - just getters

    @Override
    public double getMV() {
        return pot.getVoltage();
    }

    @Override
    public double getAngleD() {
        return poly.interpolate(getMV(), telemetry);
    }

    @Override
    public double getAngleR() {
        return getAngleD()*scaleRad;
    }

    @Override
    public boolean isAvailable() {
        return pot != null;
    }

    @Override
    public boolean isZero(){
        return getMV() <= 0.03;
    }

}
