package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.servos;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;

public class ServoArmController {

    public static class Anchor{
        public Vector2d up;
        public Vector2d right;
        public Vector2d translation;

        public Anchor(){
            this.up = new Vector2d(0, 1);
            this.right = new Vector2d(1, 0);
            this.translation = new Vector2d();
        }

        public Anchor(Vector2d up){
            this.up = up.normalized();
            this.right = new Vector2d(this.up.y, -this.up.x);
            this.translation = new Vector2d();
        }

        public Anchor(Vector2d translation, Vector2d up){
            this.up = up.normalized();
            this.right = new Vector2d(this.up.y, -this.up.x);
            this.translation = translation;
        }

        public Vector2d toRelative(Vector2d input){
            Vector2d untranslated = input.minus(translation);
            return new Vector2d(untranslated.times(right), untranslated.times(up));
        }

        public void rotateNormals(Matrix2d rotationMatrix){
            Vector2d cache = up;
            up = rotationMatrix.times(cache);
            cache = right;
            right = rotationMatrix.times(cache);
        }
    }

    private double segmentALength, segmentBLength, totalArmLength;
    private Anchor anchorA, anchorB, anchorC;
    private ServoFTC servoA, servoB, servoC;
    private ServoAngleConversion angleConversionA, angleConversionB, angleConversionC;

    public ServoArmController(Vector2d orientationA, Vector2d orientationB, Vector2d orientationC, double segmentALength, double segmentBLength, ServoFTC servoA, ServoFTC servoB, ServoFTC servoC, ServoAngleConversion angleConversionA, ServoAngleConversion angleConversionB, ServoAngleConversion angleConversionC){
        //set virtual "joints" to calculated positions
        this.anchorA = new Anchor(orientationA);
        this.anchorB = new Anchor(anchorA.up.times(segmentALength), orientationB);
        this.anchorC = new Anchor(anchorB.up.times(segmentBLength), orientationC);

        //setup arm length constants
        this.segmentALength = Math.abs(segmentALength);
        this.segmentBLength = Math.abs(segmentBLength);
        this.totalArmLength = this.segmentALength + this.segmentBLength;

        //sets the servos up
        this.servoA = servoA;
        this.servoB = servoB;
        this.servoC = servoC;

        //sets up the angle converters
        this.angleConversionA = angleConversionA;
        this.angleConversionB = angleConversionB;
        this.angleConversionC = angleConversionC;
    }

    public void calculateViaPropagation(Vector2d target, Vector2d endHeading){
        //store the length
        double targetLength = target.length();
        //stores the restricted target
        Vector2d restrictedTarget = targetLength <= totalArmLength ? target : target.normalized().times(totalArmLength);
        //stores the end heading
        Vector2d normalizedHeading = endHeading.length() <= 0.0000001 ? new Vector2d(0, -1) : endHeading.normalized();

        //set up relative variables
        double relativeAngleA, relativeAngleB, relativeAngleC;
        Matrix2d rotA, rotB;

        relativeAngleA = Math.acos(anchorA.up.times(restrictedTarget.div(targetLength))) - lawOfCosines(segmentALength, segmentBLength, targetLength);
        relativeAngleA *= Math.signum(restrictedTarget.times(anchorA.right)) >= 0 ? 1 : -1;
        rotA = Matrix2d.makeRotation(relativeAngleA);
        angleConversionA.angle2Scalar(relativeAngleA + (Math.PI / 2));
        servoA.setPosition(!angleConversionA.isOutOfRange() ? angleConversionA.getOutput() : 0);

        //rotate Servo B
        anchorB.translation = rotA.times(anchorB.translation);
        anchorB.rotateNormals(rotA);

        //rotate Servo C
        anchorC.translation = rotA.times(anchorC.translation);
        anchorC.rotateNormals(rotA);

        Vector2d targetRelativeToC = anchorB.toRelative(restrictedTarget);
        targetRelativeToC.normalize();
        relativeAngleB = Math.acos(targetRelativeToC.times(anchorB.up));
        relativeAngleB *= Math.signum(targetRelativeToC.times(anchorB.right)) >= 0 ? 1 : -1;
        rotB = Matrix2d.makeRotation(relativeAngleB);
        angleConversionB.angle2Scalar(relativeAngleB + (3 * Math.PI)/4);
        servoB.setPosition(!angleConversionB.isOutOfRange() ? angleConversionB.getOutput() : 0);

        //rotate Servo C
        anchorC.translation = rotB.times(anchorC.translation.minus(anchorB.translation)).plus(anchorB.translation);
        anchorC.rotateNormals(rotB);
        relativeAngleC = Math.acos(anchorC.up.times(normalizedHeading));
        relativeAngleC *= Math.signum(anchorC.right.times(normalizedHeading)) >= 0 ? 1 : -1;
        angleConversionC.angle2Scalar(relativeAngleC + (3 * Math.PI)/4);
        servoC.setPosition(!angleConversionC.isOutOfRange() ? angleConversionC.getOutput() : 0);
    }

    protected double lawOfCosines(double adjacentLength1, double adjacentLength2, double oppositeLength){
        return Math.acos((adjacentLength1 * adjacentLength1 + adjacentLength2 * adjacentLength2 - oppositeLength * oppositeLength) / (2 * adjacentLength1 * adjacentLength2));
    }

    public void calculateByCosines(Vector2d target, Telemetry telemetry){
        //store the length
        double targetLength = target.length();
        //stores the restricted target
        Vector2d restrictedTarget = targetLength <= totalArmLength ? target : target.normalized().times(totalArmLength);
        //set up relative variables
        double relativeAngleA, relativeAngleB, relativeAngleC;

        relativeAngleA = Math.acos(anchorA.up.times(restrictedTarget.div(targetLength))) - lawOfCosines(segmentALength, segmentBLength, targetLength);
        relativeAngleA *= Math.signum(restrictedTarget.times(anchorA.right)) >= 0 ? 1 : -1;

        relativeAngleB = lawOfCosines(segmentALength, segmentBLength, restrictedTarget.length());
        relativeAngleB *= Math.signum(restrictedTarget.times(anchorA.right)) >= 0 ? 1 : -1;

        relativeAngleC = -1*(relativeAngleA+relativeAngleB);

        angleConversionA.angle2Scalar(relativeAngleA + (Math.PI / 2));
        angleConversionB.angle2Scalar(relativeAngleB + (3 * Math.PI)/4);
        angleConversionC.angle2Scalar(relativeAngleC + (3 * Math.PI)/4);

        telemetry.addData("angle A: ", (!angleConversionA.isOutOfRange() ? angleConversionA.getOutput() : 0));
        telemetry.addData("angle B: ", (!angleConversionB.isOutOfRange() ? angleConversionB.getOutput() : 0));
        telemetry.addData("angle B: ", (!angleConversionC.isOutOfRange() ? angleConversionC.getOutput() : 0));

        /*
        servoA.setPosition(!angleConversionA.isOutOfRange() ? angleConversionA.getOutput() : 0);
        servoB.setPosition(!angleConversionB.isOutOfRange() ? angleConversionB.getOutput() : 0);
        servoC.setPosition(!angleConversionC.isOutOfRange() ? angleConversionC.getOutput() : 0);

         */
    }
}
