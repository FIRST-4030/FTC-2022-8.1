package org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors;

/**
 * Interface encapsulates the custom implementations of mathematical Vectors, storing them as points on a cartesian coordinate plane
 * <br>To make this into a generic parameter in Kotlin, please use {@code GENERIC_NAME : EULVector<*>} even though it violates finite bounds; in Java, it can ignore type parameters
 * @param <T>
 */
public interface EULVector<T extends EULVector> {

    /**
     * Method returns a string representation of the components of the vector
     * @return
     */
    String toString();

    /**
     * Method returns an array filled with the components of the vector
     * @return
     */
    double[] asArray();

    /**
     * Although the components are marked as public, this maps integer indexes to their respective components and returns them
     * @param n
     * @return
     */
    double get(int n);

    /**
     * This method is a parity of the {@code get()} method where it changes the component to the value {@code v} instead of retrieving it
     * @param n
     * @param v
     */
    void set(int n, double v);

    /**
     * Method returns the length of the vector in the space it has been defined in (2d, 3d, 4d...)
     * @return
     */
    double length();

    /**
     * Method divides all components by its length, resulting in the length of one with directional information preserved
     */
    void normalize();

    /**
     * Method returns what the vector would look like if it had been normalized (vector divided by its length)
     * @return
     */
    T normalized();

    /**
     * Method returns the sum of the vectors
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T plus(Object other);

    /**
     * Method returns the sum of the vector and the negative of the other vector
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T minus(Object other);

    /**
     * Method returns the vector * -1; negative of all components
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @return
     */
    T unaryMinus();

    /**
     * Method returns the dot product between the vectors {@code other} and the vector calling the method
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    double times(Object other);

    /**
     * Method returns the vector scaled by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T times(int other);

    /**
     * Method returns the vector scaled by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T times(float other);

    /**
     * Method returns the vector scaled by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T times(double other);

    /**
     * Method returns the vector divided by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T div(int other);

    /**
     * Method returns the vector divided by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T div(float other);

    /**
     * Method returns the vector divided by the factor {@code other}
     * <br>Even though there is no operator overloading in Java, there is in Kotlin
     * @param other
     * @return
     */
    T div(double other);
}
