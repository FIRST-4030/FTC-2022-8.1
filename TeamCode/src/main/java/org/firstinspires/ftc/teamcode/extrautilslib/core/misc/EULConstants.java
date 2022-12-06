package org.firstinspires.ftc.teamcode.extrautilslib.core.misc;

public class EULConstants {

    //NANOSECONDS to X conversions coefficients
    public static final double NANO2MS = 1e-6;
    public static final double NANO2SEC = 1e-9;

    //MILLISECONDS to X conversions coefficients
    public static final double MS2NANO = 1e6;
    public static final double MS2SEC = 1e-3;

    //SECONDS to X conversion coefficients
    public static final double SEC2NANO = 1e9;
    public static final double SEC2MS = 1e3;

    //meters to X conversion coefficients
    public static final double METERS2INCHES = 39.3701;
    public static final double METERS2FEET = 3.28084;

    //feet to X conversion coefficients
    public static final double FEET2INCHES = 12;
    public static final double FEET2METERS = 0.3048;

    //inches to X conversion coefficients
    public static final double INCHES2FEET = 1d/12d;
    public static final double INCHES2METERS = 0.0254;

    //mathematical constants that are useful
    public static final double PI = Math.PI;
    public static final double TAU = Math.PI * 2;
    public static final double GOLDEN_RATIO = (1 + Math.sqrt(5))/2;

    //mathematical conversion
    public static final double DEG2RAD = PI / 180;
    public static final double RAD2DEG = 180 / PI;
}
