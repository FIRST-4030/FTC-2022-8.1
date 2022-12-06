package org.firstinspires.ftc.teamcode.drives.swerve;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class PID<T> {

    private T setPoint;
    private T feedBack;

    //errors to keep track of
    private double lastTime, cumErr, lastErr;

    //tuning constants
    private final double Kp, Ki, Kd;

    //output
    private Telemetry telemetry;

    protected PID(Telemetry telemetry, double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.telemetry = telemetry;

        //init everything
        lastTime = System.currentTimeMillis();
        cumErr = 0;
        lastErr = 0;
    }

    //correction calculations and visualization
    public double PIDloop(){

        double elapsed = System.currentTimeMillis() - lastTime;

        //how far off is it
        double err = (double)feedBack - (double)setPoint;

        double ear = Math.abs(err);

        if(ear>Math.PI/2){
            if(ear<Math.PI){
                ear = Math.PI-ear;
            } else {
                ear = (ear+Math.PI)%(2*Math.PI);
                err*=-1;
            }

            if(ear>Math.PI/2){
                ear = Math.PI-ear;
            }
        }

        ear *= Integer.signum((int)err);
        err = ear;

        telemetry.addData("err", err);

        //proportional part
        double correction_p = err*Kp;
        telemetry.addData("Correction_P", correction_p);

        //integral part - if it gets stuck slightly off
        cumErr += err*elapsed;
        double correction_i = cumErr*Ki;
        telemetry.addData("Correction_I",correction_i);

        //derivative part - overshot correction
        double correction_d = Kd*((err - lastErr)/elapsed);
        telemetry.addData("Correction_D", correction_d);

        //keep track of errors for next loop
        lastErr = err;
        lastTime = System.currentTimeMillis();

        //output correction value
        return correction_p + correction_i + correction_d;
    }

    //getter for set point
    protected T getSetPoint() {
        return setPoint;
    }

    //set the set point
    protected void setSetPoint(T setPoint) {
        this.setPoint = setPoint;
    }

    //get feedback
    protected T getFeedBack() {
        return feedBack;
    }

    //set feedback
    protected void setFeedBack(T feedBack) {
        this.feedBack = feedBack;
    }

}
