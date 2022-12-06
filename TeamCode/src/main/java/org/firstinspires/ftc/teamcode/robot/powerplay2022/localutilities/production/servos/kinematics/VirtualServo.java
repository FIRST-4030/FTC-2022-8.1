package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;

import java.util.ArrayList;

/**
 * This class is mainly made for represent a virtual servo and an easy way to make forward kinematics
 */
public class VirtualServo {
    public Vector2d forward, right, position, armDirection, armDirectionNormal, offset;
    public double armLength;
    public ArrayList<VirtualServo> childServo;
    public VirtualServo parentServo;

    public VirtualServo(double armLength, Vector2d forward, Vector2d worldPosition){
        this.childServo = new ArrayList<>();
        this.parentServo = null;
        this.forward = forward.normalized();
        this.right = new Vector2d(this.forward.y, -this.forward.x);
        this.position = worldPosition;
        this.armDirection = this.forward;
        this.armDirectionNormal = this.right;
        this.offset = new Vector2d(0, 0);
        this.armLength = armLength;
    }

    public VirtualServo(VirtualServo parentServo, Vector2d offset, double segmentLength){
        parentServo.childServo.add(this);
        this.childServo = new ArrayList<>();
        this.parentServo = parentServo;
        this.position = parentServo.armDirection.times(this.parentServo.armLength).plus(parentServo.right.times(offset.x)).plus(parentServo.forward.times(offset.y)).plus(parentServo.position);
        this.forward = parentServo.armDirection;
        this.right = new Vector2d(this.forward.y, -this.forward.x);
        this.armDirection = parentServo.armDirection;
        this.armDirectionNormal = new Vector2d(this.armDirection.y, -this.armDirection.x);
        this.offset = offset;
        this.armLength = segmentLength;
    }

    private VirtualServo(VirtualServo servo){
        this.forward = servo.forward;
        this.right = servo.right;
        this.position = servo.position;
        this.childServo = servo.childServo;
        this.parentServo = servo.parentServo;
        this.armDirection = servo.armDirection;
        this.armDirectionNormal = new Vector2d(servo.armDirection.y, -servo.armDirection.x);
        this.offset = servo.offset;
        this.armLength = servo.armLength;
    }

    public void rotateArm(double angle){
        Matrix2d rot = Matrix2d.makeRotation(angle); //make rotation matrix

        this.armDirection = rot.times(this.armDirection); //rotate this arm
        this.armDirectionNormal = rot.times(this.armDirectionNormal); //rotate the arm's normal

        if (this.childServo.size() != 0) { //check if you can call this method in the existing children
            for (VirtualServo servo : this.childServo) {
                servo.rotateArm(angle, true);
            }
        }
    }

    public void rotateArm(double angle, boolean rotateNormals){
        Matrix2d rot = Matrix2d.makeRotation(angle); //make rotation matrix

        this.armDirection = rot.times(this.armDirection); //rotate this arm
        this.armDirectionNormal = rot.times(this.armDirectionNormal); //rotate the arm's normal

        if (rotateNormals){ //check if you need to rotate the normals as this method is recursive
            this.forward = rot.times(this.forward);
            this.right = rot.times(this.right);

            if(this.parentServo != null){ //propagate change to virtual position
                this.position = this.parentServo.armDirection.times(this.parentServo.armLength);
                this.position.plus(parentServo.armDirectionNormal.times(this.offset.x));
                this.position.plus(parentServo.armDirection.times(this.offset.y));
            }
        }

        if (this.childServo.size() != 0) { //check if you can call this method in the existing children
            for (VirtualServo servo : this.childServo) {
                servo.rotateArm(angle, true);
            }
        }
    }

    public void setArmAngle(double angle){
        setArmAngle(angle, false);
    }

    public void setArmAngle(double angle, boolean rotateNormals){
        Matrix2d rot = Matrix2d.makeRotation(angle);

        if (rotateNormals && this.parentServo != null){ //check if you need to rotate the normals as this method is recursive
            Vector2d armPosition = asRelative(armDirection.plus(this.position));
            Vector2d armNorm = asRelative(armDirectionNormal.plus(this.position));

            this.forward = parentServo.armDirection;
            this.right = parentServo.armDirectionNormal;

            Matrix2d columnMatrix = new Matrix2d(new double[][]{
                    {this.right.x, this.forward.x},
                    {this.right.y, this.forward.y}
            });

            //propagate change to virtual position
            this.position = this.parentServo.armDirection.times(this.parentServo.armLength);
            this.position.plus(parentServo.armDirectionNormal.times(this.offset.x));
            this.position.plus(parentServo.armDirection.times(this.offset.y));

            this.armDirection = columnMatrix.times(armPosition);
            this.armDirectionNormal = columnMatrix.times(armNorm);
        } else if (!rotateNormals){
            this.armDirection = rot.times(this.forward);
            this.armDirectionNormal = rot.times(this.right);
        }

        if (this.childServo.size() != 0) { //check if you can call this method in the existing children
            for (VirtualServo servo : this.childServo) {
                servo.setArmAngle(angle, true);
            }
        }
    }

    public Vector2d asRelative(Vector2d target){
        return new Vector2d(target.minus(this.position).times(this.right), target.minus(this.position).times(this.forward));
    }

    public double getArmAngle(){
        return EULMathEx.safeACOS(this.armDirection.times(this.forward)) * -Math.signum(this.armDirection.times(this.right));
    }

    public boolean hasParent(){
        return parentServo != null;
    }

    public boolean hasChildren(){
        return childServo != null && childServo.size() > 0;
    }

    public VirtualServo copy(){
        return new VirtualServo(this);
    }
}
