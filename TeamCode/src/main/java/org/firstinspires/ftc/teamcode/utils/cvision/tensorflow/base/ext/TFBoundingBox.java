package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.ext;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

public class TFBoundingBox {

    public enum DEPTH_PRECISION{
        LOWEST,
        LOW,
        MED,
        HIGH,
        HIGHEST
    }

    public float width, height;
    public Vector2d topLeft, bottomRight;

    public double estimatedAngle;
    public float estimatedDepth;

    public float confidence;

    public String label;

    public TFBoundingBox(){
        this.width = 0;
        this.height = 0;

        this.topLeft = new Vector2d();
        this.bottomRight = new Vector2d();

        this.confidence = 0;
        this.estimatedDepth = -1;
        this.estimatedAngle = 0;
        this.label = "NULL";
    }

    public TFBoundingBox(Recognition recognition){
        this.width = recognition.getWidth();
        this.height = recognition.getHeight();

        this.topLeft = new Vector2d(recognition.getLeft(), recognition.getTop());
        this.bottomRight = new Vector2d(recognition.getRight(), recognition.getBottom());

        this.confidence = recognition.getConfidence();
        this.estimatedDepth = -1;
        this.estimatedAngle = recognition.estimateAngleToObject(AngleUnit.RADIANS);
        this.label = recognition.getLabel();
    }

    public void setEstimatedDepth(float nDepth){
        this.estimatedDepth = nDepth;
    }

    public float getDepthEstimate(TFBoundingBox master, DEPTH_PRECISION precision){
        float localMin = Math.min(width, height);
        float localMax = Math.max(width, height);

        float masterMin = Math.min(master.width, master.height);
        float masterMax = Math.max(master.width, master.height);

        boolean withinRange;
        float delta = Math.abs((masterMin / localMin) - (masterMax / localMax));

        switch (precision){
            case LOWEST:
                withinRange = delta <= 0.1;
                break;
            case LOW:
                withinRange = delta <= 0.01;
                break;
            case HIGH:
                withinRange = delta <= 0.0001;
                break;
            case HIGHEST:
                withinRange = delta <= 0.00001;
                break;
            default:
                withinRange = delta <= 0.001;
        }

        return withinRange ? masterMax / localMax * master.estimatedDepth: -1;
    }

    public void updateDepthEstimate(TFBoundingBox master, DEPTH_PRECISION precision){
        this.estimatedDepth = getDepthEstimate(master, precision);
    }

    public Vector2d getCenterPoint(){
        return topLeft.plus(bottomRight).div(2d);
    }
}
