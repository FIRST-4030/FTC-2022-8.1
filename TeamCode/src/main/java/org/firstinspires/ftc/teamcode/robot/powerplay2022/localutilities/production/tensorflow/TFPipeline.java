package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.ext.TFBoundingBox;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main.TFODBase;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain.CameraLens;

import java.util.HashMap;

public class TFPipeline {

    public enum PipelineState{
        IDLE,
        SCAN,
        CULL,
        ALIGN,
        CALCULATE,
        DONE
    }

    public enum CONE_COLOR{
        RED,
        BLUE
    }

    public enum CORRECTION{
        CONE,
        JUNCTION
    }

    public TFODBase tfodBase;

    public HashMap<String, TFBoundingBox> closestBB;

    public TFBoundingBox masterCone, masterJunction;

    public PipelineState status;

    //Servo stuff
    public boolean hasAligned, hasPickedUp, hasScored;
    public double elapsedTime;
    public double armX, armY, clawClose = 0.7, clawOpen = 0.07;
    public TFBoundingBox chosen;
    public Vector2d targetPos;

    public double[] C270 = CameraLens.C270_FOV;

    public double maxSizePX = 1280, widthCone = 10, widthPole = 2.5, hFOV = C270[0];

    public float findDepth(double absWidth, double bbPx){
        return (float) ((absWidth/(2* Math.sin((hFOV*bbPx)/(2*maxSizePX)))) - (absWidth/2));
    }

    public TFPipeline(HardwareMap hardwareMap, String cameraName, String tensorflowModel, String[] tensorflowLabels){
        this.tfodBase = new TFODBase(hardwareMap, cameraName, tensorflowModel, tensorflowLabels);
        this.closestBB = new HashMap<>();

        for (String label: tensorflowLabels) {
            closestBB.put(label, new TFBoundingBox());
        }

        this.armX = 0;
        this.armY = 0;
        this.chosen = null;
        this.targetPos = new Vector2d();

        this.status = PipelineState.IDLE;
    }

    public void init(){
        tfodBase.opInit();
    }

    public void scan(){
        status = PipelineState.SCAN;
        tfodBase.scan();
        status = PipelineState.IDLE;
    }

    public void cull(){
        status = PipelineState.CULL;
        TFBoundingBox cachedMin = new TFBoundingBox();
        float depth;

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Blue Cone")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthCone, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Blue Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Red Cone")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthCone, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Red Cone", cachedMin);
        cachedMin = new TFBoundingBox();

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("Junction Top")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthPole, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        for (TFBoundingBox boundingBox: tfodBase.boundingBoxes.get("JunctionTop")) {
            if (cachedMin.estimatedDepth > (depth = findDepth(widthPole, boundingBox.width)) || boundingBox.estimatedDepth == -1){
                boundingBox.setEstimatedDepth(depth);
                cachedMin = boundingBox;
            }
        }

        closestBB.put("Junction Top", cachedMin);

        if(closestBB.get("Blue Cone").estimatedDepth > 30){
            //Do a thing
            closestBB.put("Blue Cone", null);
        }
        if(closestBB.get("Red Cone").estimatedDepth > 30){
            //Do a thing
            closestBB.put("Red Cone", null);
        }
        if(closestBB.get("Junction Top").estimatedDepth > 30){
            //Do a thing
            closestBB.put("Junction Top", null);
        }
    }

    public double correctionAngle(TFBoundingBox box){
        double x = box.getCenterPoint().x;
        x -= (maxSizePX/2);
        x = (2*x/maxSizePX) * hFOV/2;
        x *= Math.PI/3;
        return x;
    }

    public void correct(CORRECTION correctionMode, CONE_COLOR color, ServoFTC rotationalServo){
        double delta = 0; //can be modified to some offset
        double realPos = rotationalServo.getPosition();
        TFBoundingBox boundingBox = (correctionMode == CORRECTION.CONE ? closestBB.get(color == CONE_COLOR.BLUE ? "Blue Cone" : "Red Cone") : closestBB.get("Junction Top"));

        if (boundingBox == null) return; //abort correction if the target is out of range

        delta += correctionAngle(boundingBox);
        realPos += delta;
        rotationalServo.setPosition(realPos);
    }

    public void alignToPickUp(CONE_COLOR color, ServoFTC rotationServo){
        scan();
        cull();
        correct(CORRECTION.CONE, color, rotationServo);
    }

    public void alignToScore(ServoFTC rotationServo){
        scan();
        cull();
        correct(CORRECTION.JUNCTION, null, rotationServo);
    }

    public void pickupCone(double deltaTime, Vector2d offset, CONE_COLOR color, ThreeJointArm arm, ServoFTC rotationServo, ServoFTC clawServo){
        if(!hasPickedUp) {
            if (!hasAligned) {
                alignToPickUp(color, rotationServo);
                chosen = closestBB.get(color == CONE_COLOR.BLUE ? "Blue Cone" : "Red Cone");
                if (chosen == null) {
                    armX = Double.POSITIVE_INFINITY;
                    armY = Double.POSITIVE_INFINITY;
                } else {
                    armX = chosen.estimatedDepth;
                    armY = chosen.getCenterPoint().minus(new Vector2d(tfodBase.imgWidth / 2f, tfodBase.imgHeight / 2f)).times(0.0264583333d).y;
                    targetPos.x = armX;
                    targetPos.y = armY;
                }

                hasAligned = true;
            }

            //conditional for sanity check and error code checking
            if (armX != Double.POSITIVE_INFINITY && armY != Double.POSITIVE_INFINITY) {
                arm.circleFind(targetPos.plus(offset));
            }

            if (elapsedTime > 500 && elapsedTime < 550) {
                clawServo.setPosition(clawClose);
            } else if (elapsedTime > 550 && elapsedTime < 575) {
                //TODO:Add stow cmd
            } else {
                hasPickedUp = true;
            }
        }
        elapsedTime += deltaTime;
    }

    public void scoreCone(double deltaTime, Vector2d offset, ThreeJointArm arm, ServoFTC rotationServo, ServoFTC clawServo){
        if(!hasScored) {
            if (!hasAligned) {
                alignToScore(rotationServo);
                chosen = closestBB.get("Junction Top");
                if (chosen == null) {
                    armX = Double.POSITIVE_INFINITY;
                    armY = Double.POSITIVE_INFINITY;
                } else {
                    armX = chosen.estimatedDepth;
                    armY = chosen.getCenterPoint().minus(new Vector2d(tfodBase.imgWidth / 2f, tfodBase.imgHeight / 2f)).times(0.0264583333d).y;
                    targetPos.x = armX;
                    targetPos.y = armY;
                }

                hasAligned = true;
            }

            //conditional for sanity check and error code checking
            if (armX != Double.POSITIVE_INFINITY && armY != Double.POSITIVE_INFINITY) {
                arm.circleFind(targetPos.plus(offset));
            }

            if (elapsedTime > 500 && elapsedTime < 550) {
                clawServo.setPosition(clawOpen);
            } else if (elapsedTime > 550 && elapsedTime < 575) {
                //TODO:Add stow cmd
            } else {
                hasScored = true;
            }
        }
        elapsedTime += deltaTime;
    }

    public void resetBooleans(){
        hasPickedUp = false;
        hasAligned = false;
        hasScored = false;
    }
}
