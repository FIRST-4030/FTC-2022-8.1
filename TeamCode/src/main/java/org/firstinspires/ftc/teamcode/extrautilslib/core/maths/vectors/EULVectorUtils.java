package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.quaternions.Quaternion;

public class EULVectorUtils {

    public static String formatVectorAsCoord(double... components){
        String output = "(";
        int len = components.length;
        for (int i = 0; i < len; i++) {
            output += components[i] + (i != (len - 1) ? ", " : ")");
        }
        return output;
    }

    public static <T extends EULVector> EULVector add(T a, T b){
        if (a.getClass() == b.getClass()){
            return a.plus(b);
        } else {
            throw new ArithmeticException("Class A(" + a.getClass() + ") and Class B(" + b.getClass() + ") don't match!");
        }
    }

    public static <T extends EULVector> EULVector sub(T a, T b){
        if (a.getClass() == b.getClass()){
            return a.plus(b);
        } else {
            throw new ArithmeticException("Class A(" + a.getClass() + ") and Class B(" + b.getClass() + ") don't match!");
        }
    }

    public static <T extends EULVector> EULVector mul(T a, double b){
        return a.times(b);
    }

    public static <T extends EULVector> double dotP(T a, T b){
        return a.times(b);
    }

    public static <T extends EULVector> EULVector div(T a, double b){
        return a.div(b);
    }

    public static Vector3d rotate3d(Vector3d input, Quaternion quaternion){//"active" q` * V * q; "passive" is q * V * q`
        Quaternion qConjugate = quaternion.getConjugate();

        Vector4d output = qConjugate.times(input).times(quaternion).getAsVector4d();
        return new Vector3d(output.x, output.y, output.z);
    }

    public static Vector3d rotate3d(Vector3d input, Quaternion quaternion, boolean active){//"active" q` * V * q; "passive" is q * V * q`
        Quaternion qConjugate = quaternion.getConjugate();

        Vector4d output = active ? qConjugate.times(input).times(quaternion).getAsVector4d() : quaternion.times(input).times(qConjugate).getAsVector4d();
        return new Vector3d(output.x, output.y, output.z);
    }

    public static Vector2d castTo2d(Object o){
        return o instanceof Vector2d ? (Vector2d) o : new Vector2d();
    }

    public static Vector3d castTo3d(Object o){
        return o instanceof Vector3d ? (Vector3d) o : new Vector3d();
    }

    public static Vector4d castTo4d(Object o){
        return o instanceof Vector4d ? (Vector4d) o : new Vector4d();
    }

    public static <T extends EULVector> EULVector lerp(T a, T b, double t){
        return a.plus((b.minus(a)).times(t));
    }
}
