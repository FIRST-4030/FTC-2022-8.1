package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import com.qualcomm.robotcore.util.RobotLog;

public class Polynomial {

    private double[] constants;

    public Polynomial(double[] constants){
        this.constants = constants;
    }

    public int getOrder(){
        return constants.length;
    }

    public double solve(double x){
        double out = 0;
        for(int i = 0; i < getOrder(); i++) out+=constants[i]*Math.pow(x,i);
        return out;
    }

    public double rSquare(double[] x, double[] y) throws Exception {
        double out = 0;
        if(x.length != y.length) throw new Exception("nope arfhrjd");
        for(int i = 0; i < x.length; i++)out += Math.pow(y[1]-solve(x[i]),2);
        return out;
    }


    //NOT LOOP SAFE
    //COMPUTATIONAL
    public static double[] getConstants(double[] x, double[] y, int order) throws Exception {
        if(x.length != y.length) throw new Exception("nope arfhrjd: " + x.length + " is not " + y.length);

        //compute things
        double[] sxi = new double[2*order];
        double[] syi = new double[order];
        for(int i = 0; i < x.length; i++) {
            for(int j = 0; j < 2*order;j++) sxi[j] += Math.pow(x[i], j);
            for(int j = 0; j < order; j++) syi[j] += y[i] * Math.pow(x[i], j);
        }

        //augmented matrix m+1*m m = order
        double[][] mat = new double[order][order+1];

        for(int i = 0; i < order; i++){
            for(int j = 0; j < order; j++) mat[i][j] = sxi[j+i];
        }

        for(int i = 0;i < order; i++){
            mat[i][order] = syi[i];
        }
        for(int i = 0; i < order; i++){
            double klfs = mat[i][i];
            for(int j = 0; j < order+1; j++) mat[i][j] = mat[i][j]/klfs;
            for(int j = i+1; j< order; j++) {
                double minmus = mat[j][i];
                for (int k = 0; k < order + 1; k++) mat[j][k] -= minmus * mat[i][k];
            }
        }

        for(int i = order-1; i > 0; i--){
            for(int j = i-1; j > -1; j--){
                double minmus = mat[j][i];
                for(int k = 0; k < order + 1; k++) mat[j][k] -= minmus*mat[i][k];
            }
        }
        double[] out = new double[order];
        for(int i = 0;i < order;i++) out[i] = mat[i][order];

        return out;
    }


}
