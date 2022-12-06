package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVectorImpl;

//reference this video: https://youtu.be/KPoeNZZ6H4s

/**
 * This class is for modelling a second order system to control interpolation...etc
 */
public class SecondOrderDynamics {
    private float[] prev_input; //previous input
    private float[] output, output_derivative; //output and output derivative (or velocity)
    private float k1, k2, k3; //changes how the interpolation reacts/characterizes it
    private float t_crit; //variable for time step stability

    /**
     * freq is the coefficient for how fast the system behaves
     * zeta is the dampening coefficient
     * react is the coefficient for initial response
     * @param freq
     * @param zeta
     * @param react
     * @param start
     */
    public SecondOrderDynamics(double freq, double zeta, double react, DepreciatedVectorImpl start){

        if (freq == 0) throw new IllegalArgumentException("The frequency must not be zero! Inputted: " + freq);

        this.k1 = (float) (zeta / (MathEx.pi * freq));
        this.k2 = (float) (1 / ((2 * MathEx.pi * freq) * (2 * MathEx.pi * freq)));
        this.k3 = (float) (react * zeta / (2 * MathEx.pi * freq));

        this.t_crit = (float) (0.8 * (Math.sqrt(4 * k2 + k1 * k1) - k1)); //mul by 0.8 for safety

        this.prev_input = start.getAsList();
        this.output = prev_input;
        this.output_derivative = null;
    }

    public DepreciatedVectorImpl update(float timestep, DepreciatedVectorImpl input, DepreciatedVectorImpl inputDerivative){
        float[] in = input.getAsList();
        float[] inD;
        int iterations = (int) Math.ceil(timestep / t_crit);
        float actualT = timestep / iterations;
        if (prev_input.length == in.length){
            if (inputDerivative == null){
                inD = d(s(in, prev_input), timestep);
            } else {
                inD = inputDerivative.getAsList();
            }

            for (int i = 0; i < iterations; i++) {
                output = a(output, m(output_derivative, actualT));
                output_derivative = a(m(s(a(in, m(inD, k3)), output), actualT / k2), output_derivative);
            }

            return vectorize(output);
        }

        return vectorize(in);
    }

    private DepreciatedVectorImpl vectorize(float[] fa){
        switch (fa.length){
            case 2:
                return new DepreciatedVector2F(fa);
            case 3:
                return new DepreciatedVector3F(fa);
            case 4:
                return new DepreciatedVector4F(fa);
            default:
                return null;
        }
    }


    private float[] a(float[] a, float[] b){
        float[] out = new float[Math.min(a.length, b.length)];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] + b[i];
        }
        return out;
    }

    private float[] s(float[] a, float[] b){
        float[] out = new float[Math.min(a.length, b.length)];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] + b[i];
        }
        return out;
    }

    private float[] m(float[] a, float b){
        float[] out = new float[a.length];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] * b;
        }
        return out;
    }

    private float[] d(float[] a, float b){
        float[] out = new float[a.length];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] / b;
        }
        return out;
    }
}