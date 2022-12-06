package org.firstinspires.ftc.teamcode.extrautilslib.core.misc;

public class EULBooleans {

    public static boolean hasTrue(boolean[] booleans){
        boolean output = false;
        for (boolean b: booleans) {
            if (b) output = true;
        }
        return output;
    }

    public static boolean[] invertBoolArray(boolean[] booleans){
        boolean[] output = new boolean[booleans.length];
        for (int i = 0; i < booleans.length; i++) {
            output[i] = !booleans[i];
        }
        return output;
    }

    public static boolean xor(boolean a, boolean b){
        boolean sub1 = a || b;
        boolean sub2 = !(a && b);

        return (sub1 && sub2);
    }
}
