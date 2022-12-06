package org.firstinspires.ftc.teamcode.extrautilslib.core.misc;

import java.util.Stack;

public class EULArrays {

    public static int intArraySum(int[] ints){
        int output = 0;
        for (int num: ints) {
            output += num;
        }
        return output;
    }

    public static double doubleArraySum(double[] doubles){
        double output = 0;
        for (double num: doubles) {
            output += num;
        }
        return output;
    }

    public static double[] doubleArrayNegative(double[] doubles){
        double[] output = doubles;
        for (int i = 0; i < output.length; i++) {
            output[i] *= -1;
        }
        return output;
    }

    public static <T> Stack<T> stackFlip(Stack<T> input){
        Stack<T> output = new Stack<>();
        int length = input.size();
        for (int i = 0; i < length; i++) {
            output.push(input.pop());
        }

        return output;
    }


}
