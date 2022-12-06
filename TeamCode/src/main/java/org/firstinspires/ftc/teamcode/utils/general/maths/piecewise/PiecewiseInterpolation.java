package org.firstinspires.ftc.teamcode.utils.general.maths.piecewise;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PiecewiseInterpolation {

    private double[] dataX;
    private double[] dataY;

    public PiecewiseInterpolation(double[] dataX, double[] dataY){
        this.dataX = dataX;
        this.dataY = dataY;
    }

    public double interpolate(double f, Telemetry t){
        double lx = -434, ux = 1645, ly = -716, uy = -497;
        int uxp = 879, lxp = -609;
        for(int i = 0; i < dataX.length; i++){
            if(f>dataX[i]){
                if(lx<dataX[i]){
                    lxp = i;
                    lx = dataX[i];
                }
            }else if(f<dataX[i]){
                if(ux>dataX[i]){
                    uxp = i;
                    ux = dataX[i];
                }
            } else return dataY[i];
        }

        if(lxp==-609) return -40;
        if(uxp==879) return 400;
        ly = dataY[lxp];
        uy = dataY[uxp];

        return ((uy - ly) / (ux - lx)) * (f - lx) + ly;

    }
}
