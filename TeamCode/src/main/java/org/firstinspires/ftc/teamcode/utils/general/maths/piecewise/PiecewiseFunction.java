package org.firstinspires.ftc.teamcode.utils.general.maths.piecewise;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.Collections;

class element {
    private double x, y;
    
    public element(double newX, double newY) {
        setX(newX);
        setY(newY);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double setX(double newX) {
        return x = newX;
    }

    public double setY(double newY) {
        return y = newY;
    }
}

/**
 * Function that provides a piecewise math function connecting the dots between an arbitrary number of points.
 * Various options include support for step functions, the ability to chose between high and low defaults, and limiting of inputs.
 */
public class PiecewiseFunction {
    final private ArrayList<element> elements = new ArrayList<>();
    private double defaultValue = Double.MAX_VALUE;
    private int coordSize = 0, SelectedIndex = 0;
    private boolean Clamped = false, DefaultHigh = false, ClampLimits = true;

    public boolean debug = false;

    public PiecewiseFunction() {
        reset();
    }

    void sortElements() {
        Collections.sort(elements, (o1, o2) -> Double.compare(o1.getX(), o2.getX()));
    }
    
    /**
     *  Return the average value of Y weighted by the length of X
     *  For example, given the following points (0, 0), (1, 1), (2, 1), (3, 0)
     *  This would return 2/3
     */
    public double getAverage() {
        if (!isValid()) return defaultValue;
        double total = 0.0;
        for (int i = 1; i < coordSize; i++) {
            total += (getElementY(i) + getElementY(i - 1)) / 2.0 * (getElementX(i) - getElementX(i - 1));
        }
        return total / (getLastX() - getFirstX());
    }

    /**
     * Returns the list of all X elements
     */
    public ArrayList<Double> getElementsX() {
        ArrayList<Double> xList = new ArrayList<>();
        for (int i = 0; i < elements.size(); i++ ) xList.add(elements.get(i).getX());
        return xList;
    }

    /**
     * Returns the list of all Y elements
     */
    public ArrayList<Double> getElementsY() {
        ArrayList<Double> yList = new ArrayList<>();
        for (int i = 0; i < elements.size(); i++ ) yList.add(elements.get(i).getY());
        return yList;
    }

    /**
     * Changes the list of elements to have newN elements.
     * Passed newN, the number of elements that should result.
     * If elements are added, they are set to defaultValue.
     * If elements are removed, they are removed from the end of the elements list.
     */
    public void setCoordSize(int newN) {
        while (elements.size() < newN) elements.add(new element(defaultValue, defaultValue));
        while (elements.size() > newN) elements.remove(elements.size() - 1);
        coordSize = newN;
        sortElements();
    }

    /**
     * If elementNumber is valid, sets the corresponding element to the values passed.
     * If elementNumber is invalid (negative or too large), nothing happens.
     */
    public void setElement(int elementNumber, double newX, double newY) {
        if (elementNumber >= 0 && elementNumber < elements.size()) {
            elements.set(elementNumber, new element(newX, newY));
        }
        sortElements();
    }

    /**
     * Inserts a new element at the end of the existing element list.
     */
    public void addElement(double newX, double newY) {
        elements.add(new element(newX, newY));
        coordSize = elements.size();
        sortElements();
    }

    /**
     * Returns the value of the elementNumber X-element
     */
    public double getElementX(int elementNumber) {
        return elements.get(elementNumber).getX();
    }

    /**
     * Returns the value of the elementNumber Y-element
     */
    public double getElementY(int elementNumber) {
        return elements.get(elementNumber).getY();
    }

    /**
     * Returns the value of the last X element
     */
    public double getLastX() {
        if (isValid()) return elements.get(elements.size() - 1).getX();
        return defaultValue;
    }

    /**
     * Returns the value of the last Y element
     */
    public double getLastY() {
        if (isValid()) return elements.get(elements.size() - 1).getY();
        return defaultValue;
    }

    /**
     * Returns the value of the first X element
     * Equivalent to getElementX(0).
     */
    public double getFirstX() {
        if (isValid()) return elements.get(0).getX();
        return defaultValue;
    }

    /**
     * Returns the value of the first X element
     * Equivalent to getElementY(0).
     */
    public double getFirstY() {
        if (isValid()) return elements.get(0).getY();
        return defaultValue;
    }

    /**
     * Remove the indicated element from the list of elements.
     * Only takes effect if there are >2 elements.
     * If you want to reset all elements, use the reset() function.
     */
    public void removeElement(int removeIndex) {
        if (getSize() > 2) {
            elements.remove(removeIndex);
            coordSize = coordSize - 1;
        }
        sortElements();
    }

    /** Reset deletes all elements and returns things to default values.
     * DefaultValue is set to max double value.
     * All elements are deleted.
     * Clamped and defaultHigh are set to false.
     * ClampLimits and debug are set to true.
     */
    public void reset() {
        while (!elements.isEmpty()) elements.remove(0);
        defaultValue = Double.MAX_VALUE;
        coordSize = 0; SelectedIndex = 0;
        Clamped = DefaultHigh = false; ClampLimits = true;
        debug = false;
    }

    /**
     * Sets the default value that will be used when something goes wrong
     * Should be set to something recognizable.
     */
    public void setDefaultValue(double newDefault) {defaultValue = newDefault;}

    /**
     * Determines what happens if two X values exactly match each other.
     * If true, the higher of the two Y values will be used, if false, the lower of the two Y values.
     * Only matters if there are two X values that exactly match each other, such as in a square wave.
     */
    public void setDefaultHigh(boolean newDefaultHigh) {DefaultHigh = newDefaultHigh;}

    /**
     * If clampLimits is true, the received X values will be limited to [firstX, lastX].
     * If clampLimits is false, X values are treated exactly as they are and Y values are projected out.
     */
    public boolean getClampLimits() {return ClampLimits;}

    /**
     * If clampLimits is true, the received X values will be limited to [firstX, lastX].
     * If clampLimits is false, X values are treated exactly as they are and Y values are projected out.
     */
    public void setClampLimits(boolean newLimits) {ClampLimits = newLimits;}

    /**
     * Retrieve the default value.
     * Mostly so you can compare outputted Y values against it.
     */
    public double getDefaultValue() {return defaultValue;}

    /**
     * Determines what happens if two X values exactly match each other.
     * If true, the higher of the two Y values will be used, if false, the lower of the two Y values.
     * Only matters if there are two X values that exactly match each other, such as in a square wave.
     */
    public boolean getDefaultHigh() {return DefaultHigh;}

    /**
     * Returns the number of points in the elements.
     * If this function is valid, there must be two or more elements.
     */
    public int getSize() {return coordSize;}

    /**
     * Returns the segment that would be used if getY is called.
     * Value is only updated when getY is called.
     * Could be used for "stages" defined by the curve
     */
    public int getSelectedIndex() {return SelectedIndex;}

    /**
     * Returns whether clamping the X value is enabled.
     */
    public boolean clampsEnabled() {return ClampLimits;}

    /**
     * Returns true if the output is currently being clamped to a limit.
     * Also returns true if the ending segment is vertical and X > LastX OR if the starting segment is vertical and X < FirstX.
     */
    public boolean isClamped() {return Clamped;}

    /**
     * Returns TRUE if all settings are valid and providing an X will return a valid Y
     */
    public boolean isValid() {
        // If the number of UsePoints doesn't fit within the Size, that is an error.
        boolean error = (coordSize <= 1);

        // Make sure that the points given make sense
        if (!error) {
            // Check that all X values are in increasing order (IE, list of points is sorted by X value, with duplicates allowed)
            for (int i = 1; i < coordSize; i++) {
                error = error || (elements.get(i).getX() < elements.get(i - 1).getX());
            }
        }
        return !error;
    }

    /**
     * Given an X value, calculate the appropriate Y value respecting all of the various setttings.
     */
    public double getY(double X) {
        // if there is an error, return the default value;
        if (!isValid()) {
            if (debug) RobotLog.d(",debug,settings not valid! Returning DefaultValue");
            return defaultValue;
        }

        double Y = defaultValue;
        double M = defaultValue;
        double B = defaultValue;
        boolean Calc_Y = true;
        // Set several values to good defaults.
        SelectedIndex = -1;
        Clamped = false;

        if (debug) RobotLog.d(",debug,starting calculation");
        if (debug) RobotLog.d(",debug,x = " + X);
        // Check to see if X is less than the lowest X
        if (X < getFirstX()) {
            if (debug) RobotLog.d(",debug,X < first X");
            // If ClampLimits is TRUE, or the resulting line would be vertical, apply the clamp
            if (ClampLimits || elements.get(1).getX() == elements.get(0).getX()) {
                if (debug) RobotLog.d(",debug,clamping at the start");
                Y = elements.get(0).getY();
                Calc_Y = false;
                Clamped = true;
            } else {// ClampLimits is not TRUE, calculate M and B from the first pair of points
                if (debug) RobotLog.d(",debug,pre-range linear projection");
                SelectedIndex = 1;
                Calc_Y = true;
            }
        }

        // Check to see if X falls exactly on a point
        // If X falls exactly on two points, apply DefaultHigh to determine which to use
        for (int i = 0; i < coordSize; i++) {
            if (X == elements.get(i).getX()) {
                if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " (" + elements.get(i).getX() + ")");

                // Check to see if X falls exactly on the next point as well
                if (i != coordSize - 1) {
                    if (X == elements.get(i + 1).getX()) {
                        if (DefaultHigh) {
                            Y = Math.max(elements.get(i).getY(), elements.get(i + 1).getY());
                            if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " AND element number " + (i + 1) + " default high");
                        } else {
                            Y = Math.min(elements.get(i).getY(), elements.get(i + 1).getY());
                            if (debug) RobotLog.d(",debug,X ("+ X +") matches element number " + i + " AND element number " + (i + 1) + " default low");
                        }
                        Calc_Y = false;
                        break;
                    }
                }
                Y = elements.get(i).getY();
                Calc_Y = false;
                break;
            }
        }

        // Check to see if X is greater than the highest X
        if (X > getLastX()) {
            if (debug) RobotLog.d(",debug,X > last X");
            // If ClampLimits is TRUE, or if the resulting line would be vertical, apply the clamp
            if (ClampLimits || (elements.get(coordSize - 1).getX() == elements.get(coordSize - 2).getX())) {
                if (debug) RobotLog.d(",debug,clamping at the end");
                Y = elements.get(coordSize - 1).getY();
                Calc_Y = false;
                Clamped = true;
            } else { // ClampLimits is not TRUE and last line is not vertical, so calculate M and B from the last pair of points
                if (debug) RobotLog.d(",debug,post-range linear projection");
                SelectedIndex = coordSize - 1;
                Calc_Y = true;
            }
        }

        // Cycle through every used point, starting at 1
        for (int i = 1; i < coordSize; i++) {
            // Check to see which pair of points X falls between
            if (X < elements.get(i).getX() && X > elements.get(i - 1).getX()) {
                SelectedIndex = i;
                if (debug) RobotLog.d(",debug,selectedIndex = " + SelectedIndex);
                break;
            }
        }

        // If the flag hasn't been reset and SelectedIndex is valid, calculate Y
        if (Calc_Y) {
            if (debug) RobotLog.d(",debug,calculating Y,Y is currently," + Y);
            if (SelectedIndex < coordSize && SelectedIndex > 0) {
                if (debug) RobotLog.d(",debug,valid selectedIndex");
                M = (elements.get(SelectedIndex).getY() - elements.get(SelectedIndex - 1).getY()) / (elements.get(SelectedIndex).getX() - elements.get(SelectedIndex - 1).getX());
                B = elements.get(SelectedIndex).getY() - M * elements.get(SelectedIndex).getX();
                Y = M * X + B;
            } else  // the Calc_Y flag is set, but an invalid index is selected
                Y = defaultValue;
        }
        if (debug) RobotLog.d(",debug,x = " + X + ",y = " + Y + ",M = " + M + ",B = " + B + ",selectedIndex = " + SelectedIndex + ",Calc_Y = " + Calc_Y);
        return Y;
    }
}
