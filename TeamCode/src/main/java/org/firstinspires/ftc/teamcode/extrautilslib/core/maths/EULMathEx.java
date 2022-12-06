package org.firstinspires.ftc.teamcode.extrautilslib.core.maths;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;

public class EULMathEx {

    public enum Axis{
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
    }

    public static double doubleClamp(double min, double max, double value){
        return Math.min(max, Math.max(min, value));
    }

    public static float floatClamp(float min, float max, float value){
        return Math.min(max, Math.max(min, value));
    }

    public static <T extends Comparable<T>> T clamp(T min, T max, T value){
        T clampUpper = max.compareTo(value) > 0 ? value : max;
        return min.compareTo(clampUpper) < 0 ? min : clampUpper;
    }

    public static double lawOfCosines(double adjacentLength1, double adjacentLength2, double oppositeLength){
        return safeACOS((adjacentLength1 * adjacentLength1 + adjacentLength2 * adjacentLength2 - oppositeLength * oppositeLength) / (2 * adjacentLength1 * adjacentLength2));
    }

    /**
     * Due to double precision errors, the interval might be out of the [-1,1] interval, so we clamp those
     * @param ratio
     * @return acos(ratio)
     */
    public static double safeACOS(double ratio){
        if (ratio >= 1) return 0;
        if (ratio <= -1) return EULConstants.PI;
        return Math.acos(ratio);
    }

    /**
     * Due to double precision errors, the interval might be out of the [-1,1] interval, so we clamp those
     * @param ratio
     * @return asin(ratio)
     */
    public static double safeASIN(double ratio){
        if (ratio >= 1) return EULConstants.PI / 2;
        if (ratio <= -1) return -EULConstants.PI / 2;
        return Math.asin(ratio);
    }

}
