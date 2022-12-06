package org.firstinspires.ftc.teamcode.utils.general.misc;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.quaternions.Quaternion;

public class Joint {


    private Joint parentJoint;
    private double[] angleALimit, angleBLimit;
    private double angleA, angleB, length;
    private Vector4d pointA, pointB;

    public Joint(Joint parentJoint, double length, double angleAUpperBound, double angleALowerBound, double angleBUpperBound, double angleBLowerBound){
        this.parentJoint = parentJoint;
        this.angleALimit = new double[]{angleALowerBound, angleAUpperBound};
        this.angleBLimit = new double[]{angleBLowerBound, angleBUpperBound};
        this.pointA = new Vector4d();
        this.pointB = new Vector4d(Math.abs(length), 0, 0, 1);
    }

    public Joint translateAll(Vector3d translate){
        //translate endpoint A
        pointA.x += translate.x;
        pointA.y += translate.y;
        pointA.z += translate.z;

        //translate endpoint B
        pointB.x += translate.x;
        pointB.y += translate.y;
        pointB.z += translate.z;
        return this;
    }

    public Joint rotateA(Quaternion axisRotation){
        Matrix4d rotation = axisRotation.getAsPointRotationMatrix();
        pointB = rotation.times(pointB.minus(pointA)).plus(pointA);
        pointB.w = 1;
        return this;
    }

    public Joint rotateB(Quaternion axisRotation){
        Matrix4d rotation = axisRotation.getAsFrameRotationMatrix();
        pointA = rotation.times(pointA.minus(pointB)).plus(pointB);
        pointA.w = 1;
        return this;
    }

    public Joint build(){
        return this;
    }
}
