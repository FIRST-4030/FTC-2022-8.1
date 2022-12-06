package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.EULVector;

public interface EULMatrix<M extends EULMatrix, V extends EULVector> {

    String toString();
    double get(int iy, int ix);
    void set(int iy, int ix, double value);
    M plus(M other);
    M minus(M other);
    M unaryMinus();
    M times(double other);
    M times(M other);
    V times(V other);
    double det();
    void transpose();
    M transposed();
    void invert();
    M inverted();
}
