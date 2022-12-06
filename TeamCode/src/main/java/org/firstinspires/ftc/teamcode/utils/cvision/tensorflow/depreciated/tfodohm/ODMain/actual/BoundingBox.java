package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain.actual;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;

public class BoundingBox {
    private float top = 0,
                  bottom = 0,
                  right = 0,
                  left = 0;

    private double estimatedAngleTo = 0;
    private AngleUnit angleUnit;

    public BoundingBox(float top, float bottom, float right, float left, double estimatedAngleTo, AngleUnit angleUnit) {
        this.top = top;
        this.bottom = bottom;
        this.right = right;
        this.left = left;
        this.estimatedAngleTo = estimatedAngleTo;
        this.angleUnit = angleUnit;
    }

    /**
     * Does as the method says.
     * @return
     */
    public float getTop() {
        return top;
    }

    /**
     * Does as the method says.
     * @param top
     */
    public void setTop(float top) {
        this.top = top;
    }

    /**
     * Does as the method says.
     * @return
     */
    public float getBottom() {
        return bottom;
    }

    /**
     * Does as the method says.
     * @param bottom
     */
    public void setBottom(float bottom) {
        this.bottom = bottom;
    }

    /**
     * Does as the method says.
     * @return
     */
    public float getRight() {
        return right;
    }

    /**
     * Does as the method says.
     * @param right
     */
    public void setRight(float right) {
        this.right = right;
    }

    /**
     * Does as the method says.
     * @return
     */
    public float getLeft() {
        return left;
    }

    /**
     * Does as the method says.
     * @param left
     */
    public void setLeft(float left) {
        this.left = left;
    }

    /**
     * Does as the method says.
     * @return
     */
    public DepreciatedVector2F getCenter(){
        return new DepreciatedVector2F((right + left) / 2, (top + bottom) / 2);
    }

    /**
     * This method returns a corner vertex based on the index passed starting on the top left corner at index 0
     *
     * CCW Vertex Ordering
     * 0--1
     * |  |
     * 3--2
     *
     * @param index
     * @return corner vector
     */
    public DepreciatedVector2F getCorner(int index){
        DepreciatedVector2F output = new DepreciatedVector2F();

        //prevents index from going out of bounds
        if ((index >= 0) && (index <= 3)) {
            output.x = index % 3 != 0 ? right : left;
            output.y = index < 2 ? top : bottom;
        }

        return output;
    }
}
