package org.firstinspires.ftc.teamcode.drives.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.fileRW.ConfigFileUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.pot.DoublePotentiometer;

import java.util.ArrayList;
import java.util.Arrays;

public class SwervePod extends PID<Double>{

    //tuning constants, 0 == not used
    private final static double Kp = 0.2;
    private final static double Ki = 0;
    private final static double Kd = 0;

    //hardware
    private final DcMotor m1, m2;
    private DoublePotentiometer pot;

    //targets
    private double targetVelocity, targetAngle;

    //variable output
    private Telemetry telemetry;


    //initialization and hardware setup
    public SwervePod(Telemetry telemetry, DcMotor m1, DcMotor m2, DoublePotentiometer pot) {
        super(telemetry, Kp, Ki, Kd);
        this.telemetry = telemetry;
        this.m1 = m1;
        this.m2 = m2;
        this.pot = pot;
        targetAngle = 0;
        targetVelocity = 0;

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //zero out the wheels
    public void tune(){

        m2.setTargetPosition(0);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setPower(0.5);
        m1.setPower(0.2);

        int spinRes = 5; // do it 5 times to find the center of each zero

        int lowerBound = -283;
        int[] zeros = new int[spinRes];
        int[] points = new int[spinRes];

        boolean cool = true;
        for(int i = 0; i < spinRes;){

            //use the offset potentiometer values to count revolutions
            if(pot.isZeroOne()) {
                int pos = m1.getCurrentPosition();
                if (lowerBound == -283) lowerBound = pos;

                zeros[i] += (pos - lowerBound);
                points[i] ++;

                cool = true;
            }
            if(pot.isZeroTwo() && cool){
                i++;
                cool = false;
            }
            //visualization
            telemetry.addData("pot1", pot.getMV1());
            telemetry.addData("pot2", pot.getMV2());
            telemetry.update();
        }

        //calculate motor encoder ticks per revolution based on data
        float tpr = 0;
        for(int i = 0;i < spinRes; i++) {
            if(points[i] != 0) tpr += (zeros[i] / points[i]);
            else spinRes--;
        }
        tpr /= spinRes;
        tpr /= 2;
        tpr = 1740; //todo

        m1.setPower(0);

        RobotLog.d("AUTO TUNE tpr: " + tpr);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setTargetPosition((int)tpr);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setPower(0.2);

        ArrayList<Double> pd1a = new ArrayList<Double>();
        ArrayList<Double> pd1b = new ArrayList<Double>();
        ArrayList<Double> pd2a = new ArrayList<Double>();
        ArrayList<Double> pd2b = new ArrayList<Double>();
        ArrayList<Double> td1a = new ArrayList<Double>();
        ArrayList<Double> td1b = new ArrayList<Double>();
        ArrayList<Double> td2a = new ArrayList<Double>();
        ArrayList<Double> td2b = new ArrayList<Double>();

        double thr1 = 0;
        double thr2 = 0;

        double tpd  = tpr/360;

        for(;m1.getCurrentPosition() < tpr;){
            telemetry.addData("pot1",m1.getCurrentPosition()/tpd);
            telemetry.addData("pot2",0);
            if(m1.getCurrentPosition()<tpr/2){
                pd2a.add(pot.getMV2());
                td2a.add(m1.getCurrentPosition()/tpd);

                telemetry.addData("pot1",1);
                thr2 = pot.getMV1();
            } else {
                pd2b.add(pot.getMV2());
                td2b.add(m1.getCurrentPosition()/tpd);

                telemetry.addData("pot1",0);
            }
            if(m1.getCurrentPosition()<tpr/4||m1.getCurrentPosition()>3*tpr/4){
                pd1a.add(pot.getMV1());
                td1a.add(m1.getCurrentPosition()/tpd);
                telemetry.addData("pot2",1);
            } else {
                pd1b.add(pot.getMV1());
                td1b.add(m1.getCurrentPosition()/tpd);
                telemetry.addData("pot2",0);
                thr1 = pot.getMV2();
            }
            telemetry.update();
        }

        try {
            RobotLog.d("AUTOTUNE poly 1: " + pd1a);//Arrays.toString(Polynomial.getConstants(pd1a, td1a,20)));
            RobotLog.d("AUTOTUNE poly 2: " + pd2a);//Arrays.toString(Polynomial.getConstants(pd2a, td2a,20)));
            RobotLog.d("AUTOTUNE poly 3: " + pd1b);//Arrays.toString(Polynomial.getConstants(pd1a, td1a,20)));
            RobotLog.d("AUTOTUNE poly 4: " + pd2b);
            RobotLog.d("AUTOTUNE cracker 1: " + td1a);RobotLog.d("AUTOTUNE cracker 2: " + td2a);
            RobotLog.d("AUTOTUNE cracker 3: " + td1b);RobotLog.d("AUTOTUNE cracker 4: " + td2b);
            RobotLog.d("AUTOTUNE thresholds: " + Arrays.toString(new double[]{thr1, thr2}));

            ArrayList<Double> thresh = new ArrayList<Double>();
            thresh.add(thr1);
            thresh.add(thr2);
            ArrayList<ArrayList<Double>> data1 = new ArrayList<ArrayList<Double>>();
            data1.add(pd1a);
            data1.add(pd2a);
            data1.add(pd1b);
            data1.add(pd2b);
            data1.add(td1a);
            data1.add(td2a);
            data1.add(td1b);
            data1.add(td2b);
            data1.add(thresh);

            ConfigFileUtil.init();
            ConfigFileUtil.writeToConfig("swerve_pod_1_profile", data1,1);

        } catch (Exception e) {
            RobotLog.d("uh oh " + e.getMessage() + " sdfsd" + Arrays.toString(e.getStackTrace()));
        }


        //reset hardware
        m1.setPower(0);
        m2.setPower(0);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void loop() {
        //get current position and set that in out correction
        setFeedBack(pot.getAngleR());//pot.getAngleD());

        //visualization
        telemetry.addData("feedBack", pot.getAngleR());

        // do fancy math and calculate correction value based on the error of the angle
        double correction = PIDloop();

        //output and visualization for each side
        m1.setPower(targetVelocity + correction);
        telemetry.addData("mot1", targetVelocity + correction);

        m2.setPower(-targetVelocity + correction);
        telemetry.addData("mot2", -targetVelocity + correction);
    }

    //setter for velocity target
    public void setTargetVelocity(double velocity){
        targetVelocity = velocity;
    }

    //setter for angle target
    public void setTargetAngle(double angle){
        targetAngle = angle;
        setSetPoint(angle);
    }

    public void hmmm(int dij){
        m1.setTargetPosition(dij);
        m2.setTargetPosition(0);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setPower(0.5);
        m2.setPower(0.5);
    }

    public void hmmmm(){
        m1.setPower(-1);
        m2.setPower(1);
    }

    //spins around - mainly for testing
    public void spin(){
        m1.setPower(0.2);
        m2.setPower(0);

        telemetry.addData("pot1MV", pot.getMV1()*360/3.3);
        telemetry.addData("pot2MV", pot.getMV2()*360/3.3);
        telemetry.addData("pot1a", pot.getAngleD1a());
        telemetry.addData("pot2a", pot.getAngleD2a());
        telemetry.addData("pot1b", pot.getAngleD1b());
        telemetry.addData("pot2b", pot.getAngleD2b());
        telemetry.addData("potT", pot.getAngleD());
        telemetry.update();
    }

    //stop
    public void brake(){
        m1.setPower(0);
        m2.setPower(0);
    }

    //when zeros out the wheel as best its able while running
    public void zero(){
        if(pot.isZeroOne() && (Math.abs(m1.getCurrentPosition()) > 10 || Math.abs(m2.getCurrentPosition()) > 10)){
            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            if(pot.getMV1() > pot.getMV2()) targetAngle += 0.01;
            else targetAngle -= 0.01;
        }
    }

}
