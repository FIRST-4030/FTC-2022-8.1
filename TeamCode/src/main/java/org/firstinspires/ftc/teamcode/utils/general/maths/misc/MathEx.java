package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;

import java.util.ArrayList;

public class MathEx {

    //important constants
    public static final double pi = Math.PI;
    public static final double e = Math.E;
    public static final double golden_ratio = (1 + Math.sqrt(5)) / 2;
    public static final double tau = Math.PI * 2;

    //quick conversions
    public static final double rad2deg = 180 / pi;
    public static final double deg2rad = pi / 180;


    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * @param a
     * @param b
     * @param t
     * @return interpolated float
     */
    public static float lerp(float a, float b, float t){
        return a + (b - a) * t;
    }

    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * <br>Vector2f
     * @param a
     * @param b
     * @param t
     * @return interpolated Vector2f
     */
    public static DepreciatedVector2F lerp(DepreciatedVector2F a, DepreciatedVector2F b, float t){
        float nX = a.getX() + (b.getX() - a.getX()) * t;
        float nY = a.getY() + (b.getY() - a.getY()) * t;

        return new DepreciatedVector2F(nX, nY);
    }

    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * <br>Vector3f
     * @param a
     * @param b
     * @param t
     * @return interpolated Vector3f
     */
    public static DepreciatedVector3F lerp(DepreciatedVector3F a, DepreciatedVector3F b, float t){
        float nX = a.getX() + (b.getX() - a.getX()) * t;
        float nY = a.getY() + (b.getY() - a.getY()) * t;
        float nZ = a.getZ() + (b.getZ() - a.getZ()) * t;

        return new DepreciatedVector3F(nX, nY, nZ);
    }

    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * <br>Vector4f
     * @param a
     * @param b
     * @param t
     * @return interpolated Vector4f
     */
    public static DepreciatedVector4F lerp(DepreciatedVector4F a, DepreciatedVector4F b, float t){
        float nX = a.getX() + (b.getX() - a.getX()) * t;
        float nY = a.getY() + (b.getY() - a.getY()) * t;
        float nZ = a.getZ() + (b.getZ() - a.getZ()) * t;
        float nW = a.getW() + (b.getW() - a.getW()) * t;

        return new DepreciatedVector4F(nX, nY, nZ, nW);
    }
    /**
     * quick clamping solution. Clamps a value between a min and max
     * @param min
     * @param max
     * @param value
     * @return clamped value
     */
    public static float clamp(float min, float max, float value){
        return Math.min(max, Math.max(min, value));
    }
    
    /**
     * Take the factorial of an input integer
     * @param n
     * @return factorial of n / n!
     */
    public static int factorialInt(int n){
        int res = 1, i;
        for (i = 2; i <= n; i++){
            res *= i;
        }
        return res;
    }

    /**
     * Returns the intersection between two given 2d line segments (Line 1 is determined by l1 & l2, while Line 2 is determined by l3 & l4).
     * It just uses Vector3f for convenience.
     * @param l1
     * @param l2
     * @param l3
     * @param l4
     * @return new Vector3f(ix, iy, 1);
     */
    public static DepreciatedVector2F llInt2d(DepreciatedVector2F l1, DepreciatedVector2F l2, DepreciatedVector2F l3, DepreciatedVector2F l4){
        float x1 = l1.getX();
        float x2 = l2.getX();

        float y1 = l1.getY();
        float y2 = l2.getY();

        float x3 = l3.getX();
        float x4 = l4.getX();

        float y3 = l3.getY();
        float y4 = l4.getY();

        float denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));

        if (denominator == 0) {return null;}

        float e1 = quick2fArrDet(new float[]{x1, y1, x2, y2});
        float e3 = quick2fArrDet(new float[]{x3, y3, x4, y4});
        float e4 = quick2fArrDet(new float[]{x3,  1, x4,  1});

        float e2x = quick2fArrDet(new float[]{x1, 1, x2, 1});
        float e2y = quick2fArrDet(new float[]{y1, 1, y2, 1});

        float ix = quick2fArrDet(new float[]{e1, e2x, e3, e4}) / denominator;
        float iy = quick2fArrDet(new float[]{e1, e2y, e3, e4}) / denominator;

        return new DepreciatedVector2F(ix, iy);
    }

    /**
     * returns the determinant of a 4 length array without casting it into a Matrix2f object
     * <br> [a,b] is arranged like {a, b, c, d} in a 1d array
     * <br> [c,d]
     *
     * @param a
     * @return a * c - b * d
     */
    public static float quick2fArrDet(float[] a){
        return a[0] * a[3] - a[1] * a[2];
    }

    /**
     * returns the determinant of a 9 length array without casting it into a Matrix3f object
     * <br> [a,b,c] is arranged like {a, b, c, d, e, f, g, h, i} in a 1d array
     * <br> [d,e,f]
     * <br> [g,h,i]
     *
     * @param a
     * @return a * |e,f,h,i| - b * |d,f,g,i| + c * |d,e,g,h|
     */
    public static float quick3fArrDet(float[] a){
        float[] min1 = {a[4], a[5], a[7], a[8]};
        float[] min2 = {a[3], a[5], a[6], a[8]};
        float[] min3 = {a[3], a[4], a[6], a[7]};
        return quick2fArrDet(min1) * a[0] - quick2fArrDet(min2) * a[1] + quick2fArrDet(min3) * a[2];
    }

    /**
     * bypass using Vector2f length method
     * @return float length
     */
    public static float quickLengthF(float x1, float y1, float x2, float y2){
        return (float) Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) - (y2 - y1)));
    }

    /**
     * bypass using Vector2f length method
     * @return double length
     */
    public static double quickLengthD(float x1, float y1, float x2, float y2){
        return Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) - (y2 - y1)));
    }
    
    /**
     * Get the cross product of two input Vector3f 
     * <br> Side Note: this is used for finding the normal vector (order does matter)
     * <br> Right hand rule: thumb points up, index forward, middle finger left; a is index, b is middle, thumb is the crossproduct of a and b
     */
    /*public static Vector3f crossProduct3f(Vector3f a, Vector3f b){
        float cx = (a.getY() * b.getZ()) − (a.getZ() * b.getY());
        float cy = (a.getZ() * b.getX()) − (a.getX() * b.getZ());
        float cz = (a.getX() * b.getY()) − (a.getY() * b.getX());
        return new Vector3f(cx, cy, cz);
    }*/

    /**
     * This method converts the sensor width, sensor height, and focal length of a camera into horizontal & vertical FOV in radians
     * @param width
     * @param height
     * @param focal_len
     * @return
     */
    public static double[] findFOV(double width, double height, double focal_len){
        double hFov, vFov;
        hFov = 2 * Math.atan2(width, 2 * focal_len);
        vFov = 2 * Math.atan2(height, 2 * focal_len);

        return new double[] {hFov, vFov};
    }

    /**
     * This method interpolates through a bezier curve through the given t value. The control points first and last element are the beginning and end points respectively.
     * @param controlPoints
     * @param t
     * @return interpolated vector2f
     */
    public static DepreciatedVector2F bezierCurveGen(ArrayList<DepreciatedVector2F> controlPoints, float t){
        ArrayList<DepreciatedVector2F> cachedPoints = controlPoints, newCachedPoints = new ArrayList<>();
        DepreciatedVector2F tempPoint = new DepreciatedVector2F();

        if (controlPoints.size() < 1){ return null;} //abort
        else if (controlPoints.size() == 1){return controlPoints.get(0);}

        //loop over points, find interpolation points, repeat
        while (cachedPoints.size() > 1) {


            for (int i = 0; i < cachedPoints.size() - 1; i++) {
                //          get point at index ; get point connected to prev ; interpolation factor
                tempPoint = lerp(cachedPoints.get(i), cachedPoints.get(i + 1), t);
                newCachedPoints.add(tempPoint);
            }

            //overwrite points
            cachedPoints = newCachedPoints;
            newCachedPoints.clear();
        }

        return cachedPoints.get(0); //the result is in a size one array, so get the first and only element of it
    }

    /**
     * This method interpolates through a quadratic bezier curve given a 't' scalar value
     * @param begin
     * @param control
     * @param end
     * @param t
     * @return output
     */
    public DepreciatedVector2F bezierCurveQ(DepreciatedVector2F begin, DepreciatedVector2F control, DepreciatedVector2F end, float t){
        DepreciatedVector2F p0p1Diff = DepreciatedVector2F.sub(begin, control);
        DepreciatedVector2F p2p1Diff = DepreciatedVector2F.sub(end, control);
        float t1Squared = (1 - t) * (1 - t);
        float tSquared = t * t;

        DepreciatedVector2F output = DepreciatedVector2F.add(control, DepreciatedVector2F.add(DepreciatedVector2F.mul(p0p1Diff, t1Squared), DepreciatedVector2F.mul(p2p1Diff, tSquared)));
        return output;
    }

    /**
     * This method interpolates through the derivative of a quadratic bezier curve given a 't' scalar value
     * @param begin
     * @param control
     * @param end
     * @param t
     * @return
     */
    public DepreciatedVector2F bezierCurveQD(DepreciatedVector2F begin, DepreciatedVector2F control, DepreciatedVector2F end, float t){
        DepreciatedVector2F p1p0Diff = DepreciatedVector2F.sub(begin, control);
        DepreciatedVector2F p2p1Diff = DepreciatedVector2F.sub(end, control);
        float t1 = 2 * (1 - t);

        DepreciatedVector2F output = DepreciatedVector2F.add(DepreciatedVector2F.mul(p1p0Diff, t1), DepreciatedVector2F.mul(p2p1Diff, 2 * t));
        return output;
    }


    public DepreciatedVector2F[] bezierCurveQArray(DepreciatedVector2F begin, DepreciatedVector2F control, DepreciatedVector2F end, int steps){
        DepreciatedVector2F p0p1Diff = DepreciatedVector2F.sub(begin, control);
        DepreciatedVector2F p2p1Diff = DepreciatedVector2F.sub(end, control);
        DepreciatedVector2F[] output = new DepreciatedVector2F[steps];

        float incr = 1 / steps;
        float t = 0, t1Squared, tSquared;

        for (int i = 0; i < steps; i++) {
            t1Squared = (1 - t) * (1 - t);
            tSquared = t * t;

            output[i] = DepreciatedVector2F.add(control, DepreciatedVector2F.add(DepreciatedVector2F.mul(p0p1Diff, t1Squared), DepreciatedVector2F.mul(p2p1Diff, tSquared)));

            t += incr;
        }


        return output;
    }

    /**
     * Make an arc from (0,0) and the given point / Generate the first 2 parameters of the arcTo method
     * <br>the output is [0] = radius; [1] = arc length
     * @param target
     * @return
     */
    public static double[] makeArcV1(DepreciatedVector2F target){
        DepreciatedVector2F int2f = target;
        DepreciatedVector2F targetMid = DepreciatedVector2F.mul(target, 0.5f);

        //get the direction that the point from (0,0)
        DepreciatedVector2F int2fDir = int2f.normalized();

        //find the normals through the direction (both a 90 & 270 rotation of int2fDir)
        DepreciatedVector2F int2fNorm90 = new DepreciatedVector2F(-int2fDir.getY(), int2fDir.getX());
        DepreciatedVector2F int2fNorm270 = new DepreciatedVector2F(int2fDir.getY(), -int2fDir.getX());

        //compare the direction of the normals using the dot product
        DepreciatedVector2F closestToXAxis = int2fDir.getX() <= 0 ? (int2fDir.getY() >= 0 ? int2fNorm90 : int2fNorm270) : (int2fDir.getY() <= 0 ? int2fNorm90 : int2fNorm270);
        closestToXAxis.mul(1000);
        closestToXAxis.add(targetMid);

        DepreciatedVector2F output = MathEx.llInt2d(new DepreciatedVector2F(int2f.length() * 1000, 0), new DepreciatedVector2F(int2f.length() * -1000,0), closestToXAxis, targetMid);

        assert output != null;
        DepreciatedVector2F diff = DepreciatedVector2F.sub(int2f, output);

        double angle = Math.atan2(diff.getY(), diff.getX());

        return new double[]{-output.getX(), ((angle > pi ? -1 * (2 * pi - angle) : angle) * 2 * Math.abs(output.getX()))};
    }


    /**
     * Make an arc from (0,0) and the given point / Generate the first 2 parameters of the arcTo method
     * <br>the output is [0] = radius; [1] = arc length
     * <br>V2
     * @param target
     * @return
     */
    public static float[] makeArcV2(DepreciatedVector2F target){
        double length = target.length();
        double alpha = Math.atan2(target.getX(), target.getY());
        double beta = (pi/2) - alpha;
        double radius = (length / 2) / Math.cos(alpha);
        double arcLen = radius * beta * 2;

        return new float[]{ (float) radius, (float) arcLen};
    }

    public static float[] makeArcV3(DepreciatedVector2F target){
        double length = target.length();
        double sign = Math.signum(target.getX());
        double theta = Math.atan2(Math.abs(target.getY()), Math.abs(target.getX()));
        double radius = (length / Math.cos(theta)) * -sign;
        double arcLength = (2 * radius * ((pi/2) - theta));

        return new float[]{(float) radius, (float) arcLength};
    }

    public static float[] quickFArrayAdd(float[] a, float[] b){
        float[] out = new float[Math.min(a.length, b.length)];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] + b[i];
        }
        return out;
    }

    public static float[] quickFArraySub(float[] a, float[] b){
        float[] out = new float[Math.min(a.length, b.length)];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] - b[i];
        }
        return out;
    }

    public static float[] quickFArraySMul(float[] a, float b){
        float[] out = new float[a.length];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] * b;
        }
        return out;
    }

    public static float[] quickFArraySDiv(float[] a, float b){
        float[] out = new float[a.length];
        for (int i = 0; i < out.length; i++){
            out[i] = a[i] / b;
        }
        return out;
    }
}
