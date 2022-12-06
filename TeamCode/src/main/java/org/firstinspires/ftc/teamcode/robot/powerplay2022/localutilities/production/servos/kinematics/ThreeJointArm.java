package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;

//This class is a system to control an arm with three joints with IK as the approach
public class ThreeJointArm {

    private VirtualServo virtualServoA, virtualServoB, virtualServoC;
    private ServoFTC servoA, servoB, servoC;
    private final AngleConversion conversionA, conversionB, conversionC;
    private final double armLengthA, armLengthB, totalArmLength;
    private Telemetry telemetry;

    public ThreeJointArm(VirtualServo virtualCore, ServoFTC[] servos, AngleConversion[] conversions, double armLengthA, double armLengthB){
        //check if params are correct
        if (servos.length != 3) throw new IllegalArgumentException("Servo Array is not length 3! Length passed in: " + servos.length);
        if (conversions.length != 3) throw new IllegalArgumentException("Conversion Array is not length 3! Length passed in: " + conversions.length);

        //assign segment lengths
        this.armLengthA = armLengthA;
        this.armLengthB = armLengthB;
        this.totalArmLength = armLengthA + armLengthB;

        //assign conversions
        this.conversionA = conversions[0];
        this.conversionB = conversions[1];
        this.conversionC = conversions[2];

        //assign hardware servos
        this.servoA = servos[0];
        this.servoB = servos[1];
        this.servoC = servos[2];

        //assign virtual servos
        this.virtualServoA = virtualCore;
        this.virtualServoB = new VirtualServo(this.virtualServoA, new Vector2d(), armLengthB);
        this.virtualServoC = new VirtualServo(this.virtualServoB, new Vector2d(), 0);
    }

    public ThreeJointArm(VirtualServo virtualCore, ServoFTC servo, AngleConversion[] conversions, double armLengthA, double armLengthB){

        //assign segment lengths
        this.armLengthA = armLengthA;
        this.armLengthB = armLengthB;
        this.totalArmLength = armLengthA + armLengthB;

        //assign conversions
        this.conversionA = conversions[0];
        this.conversionB = conversions[1];
        this.conversionC = conversions[2];

        //assign hardware servos
        this.servoC = servo;

        //assign virtual servos
        this.virtualServoA = virtualCore;
        this.virtualServoB = new VirtualServo(this.virtualServoA, new Vector2d(), armLengthB);
        this.virtualServoC = new VirtualServo(this.virtualServoB, new Vector2d(), 0);
    }

    public void bindTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    public void circleFind(Vector2d target){
        Vector2d restrictedTarget = target.length() <= (totalArmLength-0.5) ? target : target.normalized().times(totalArmLength);
        double b = (armLengthA*armLengthA - armLengthB*armLengthB - restrictedTarget.length()*restrictedTarget.length())/(-2*restrictedTarget.length());
        double angleToTarget = EULMathEx.safeACOS(-1 * restrictedTarget.x/restrictedTarget.length());
        double a = restrictedTarget.length() - b;
        double h = Math.sqrt(armLengthB*armLengthB - b*b);
        //Why is A this and not: double A = EULMathEx.safeASIN(h/armLengthA);
        double A = EULMathEx.safeASIN(restrictedTarget.y/restrictedTarget.length()) + EULMathEx.safeASIN(h/armLengthA);
        double B = EULMathEx.safeASIN(a/armLengthA) + EULMathEx.safeASIN(b/armLengthB);
        double C = (Math.PI*1.75 - A - B)/(Math.PI*1.5);
        telemetry.addData("Angle A Pi Rad: ", A/Math.PI);
        telemetry.addData("Angle B Pi Rad: ", B/Math.PI);
        A=A/(Math.PI);
        B=B/(Math.PI);
        telemetry.addData("Angle B Output Raw: ", B);
        telemetry.addData("Angle A Output Raw: ", A);
        telemetry.addData("Angle A Output: ", EULMathEx.doubleClamp(0.001, 0.999, A));
        //servoB.setPosition(EULMathEx.doubleClamp(0.001, 0.999, B));
        //servoC.setPosition(EULMathEx.doubleClamp(0.001, 0.999, C-0.3));
        if(A>=1){
            A=0.99;
        }
        if(B>=0.99){
            B=EULMathEx.doubleClamp(0.001, 0.999, B);
        }
        if(Double.isNaN(A)){A=0.5;}
        if(Double.isNaN(B)){B=1;}
        if(Double.isNaN(C)){C=(Math.PI*1.75 - A - B)/(Math.PI*1.5);}
        if(Double.isNaN(C)){C=0;}
        servoA.setPosition(EULMathEx.doubleClamp(0.001, 0.999, A + 0.244));
        servoB.setPosition(EULMathEx.doubleClamp(0.001, 0.999, B - 0.07));
        servoC.setPosition(EULMathEx.doubleClamp(0.001, 0.999, C-0.4));
        //servoA.setPosition(0.5);
        //servoB.setPosition(1);
        //servoC.setPosition(0.5);
        telemetry.addData("Restricted Target: ", restrictedTarget);
        telemetry.addData("A: ", A*2);
        telemetry.addData("B: ", B*2);
        telemetry.addData("A Real: ", servoA.getPosition());
        telemetry.addData("B Real: ", servoB.getPosition());
    }

    public void circleFindArmTwo(Vector2d target){
        Vector2d restrictedTarget = target.length() <= (totalArmLength-0.5) ? target : target.normalized().times(totalArmLength);
        double b = (armLengthA*armLengthA - armLengthB*armLengthB - restrictedTarget.length()*restrictedTarget.length())/(-2*restrictedTarget.length());
        double angleToTarget = EULMathEx.safeACOS(-1 * restrictedTarget.x/restrictedTarget.length());
        double a = restrictedTarget.length() - b;
        double h = Math.sqrt(armLengthB*armLengthB - b*b);
        //Why is A this and not: double A = EULMathEx.safeASIN(h/armLengthA);
        double A = EULMathEx.safeASIN(restrictedTarget.y/restrictedTarget.length()) + EULMathEx.safeASIN(h/armLengthA);
        double B = EULMathEx.safeASIN(a/armLengthA) + EULMathEx.safeASIN(b/armLengthB);
        double C = (Math.PI*1.75 - A - B)/(Math.PI*1.5);
        telemetry.addData("Angle A Pi Rad: ", A/Math.PI);
        telemetry.addData("Angle B Pi Rad: ", B/Math.PI);
        A=A/(Math.PI);
        B=B/(Math.PI);
        telemetry.addData("Angle B Output Raw: ", B);
        telemetry.addData("Angle A Output Raw: ", A);
        telemetry.addData("Angle A Output: ", EULMathEx.doubleClamp(0.001, 0.999, A));
        //servoB.setPosition(EULMathEx.doubleClamp(0.001, 0.999, B));
        //servoC.setPosition(EULMathEx.doubleClamp(0.001, 0.999, C-0.3));
        if(A>=1){
            A=0.99;
        }
        if(B>=0.99){
            B=EULMathEx.doubleClamp(0.001, 0.999, B);
        }
        if(Double.isNaN(A)){A=0.5;}
        if(Double.isNaN(B)){B=1;}
        if(Double.isNaN(C)){C=(Math.PI*1.75 - A - B)/(Math.PI*1.5);}
        if(Double.isNaN(C)){C=0;}
        servoA.setPosition(EULMathEx.doubleClamp(0.001, 0.999, A + 0.244));
        servoB.setPosition(EULMathEx.doubleClamp(0.001, 0.999, B - 0.07));
        servoC.setPosition(EULMathEx.doubleClamp(0.05, 0.999, C-0.2));
        //servoA.setPosition(1);
        //servoB.setPosition(0.5);
        //servoC.setPosition(0);
        telemetry.addData("Restricted Target: ", restrictedTarget);
        telemetry.addData("A: ", A*2);
        telemetry.addData("B: ", B*2);
        telemetry.addData("A Real: ", servoA.getPosition());
        telemetry.addData("B Real: ", servoB.getPosition());
    }

    public void propagate(Vector2d target, Vector2d endHeading, boolean bottomSolution){
        //store the length
        double targetLength = target.length();
        //stores the restricted target and direction to it
        Vector2d restrictedTarget = targetLength <= totalArmLength ? target : target.normalized().times(totalArmLength);
        Vector2d targetDir = restrictedTarget.normalized();


        //stores the end heading
        Vector2d normalizedHeading = endHeading.length() <= 0.0000001 ? new Vector2d(0, -1) : endHeading.normalized();

        /*
        double biasedSign = Math.signum(virtualServoA.armDirectionNormal.times(targetDir)) == 0 ? 1 : Math.signum(virtualServoA.armDirectionNormal.times(targetDir));
        double angleAVirtual = Math.acos(virtualServoA.armDirection.times(targetDir)) * biasedSign; //angle from the arm's virtual base

        double angleA = Math.acos(virtualServoA.forward.times(targetDir));
        if (bottomSolution){
            angleA -= EULMathEx.lawOfCosines(this.armLengthA, targetLength, armLengthB);
        } else {
            angleA += EULMathEx.lawOfCosines(this.armLengthA, targetLength, armLengthB);
        }

        //rotate and actuate virtual and hardware servo (respectively)
        conversionA.angle2Scalar(angleA + (Math.PI / 2));
        virtualServoA.rotateArm(angleAVirtual);
        servoA.setPosition(conversionA.getOutput());

        Vector2d relativeBTarget = restrictedTarget.minus(virtualServoB.position).normalized();

        double biasedSignBVirtual = Math.signum(virtualServoB.armDirectionNormal.times(relativeBTarget)) == 0 ? 1 : Math.signum(virtualServoB.armDirectionNormal.times(relativeBTarget));
        double biasedSignBActual = Math.signum(virtualServoB.right.times(relativeBTarget)) == 0 ? 1 : Math.signum(virtualServoB.right.times(relativeBTarget));
        double angleBVirtual = Math.acos(virtualServoB.armDirection.times(relativeBTarget)) * biasedSignBVirtual;
        double angleBActual = Math.acos(virtualServoB.forward.times(relativeBTarget)) * biasedSignBActual;

        //rotate and actuate virtual and hardware servo (respectively)
        conversionB.angle2Scalar(3 * Math.PI / 4 - angleBActual);
        virtualServoB.rotateArm(angleBVirtual);
        servoB.setPosition(conversionB.getOutput());

        double biasedSignCVirtual = Math.signum(virtualServoB.armDirectionNormal.times(normalizedHeading)) == 0 ? 1 : Math.signum(virtualServoB.armDirectionNormal.times(normalizedHeading));
        double biasedSignCActual = Math.signum(virtualServoB.right.times(normalizedHeading)) == 0 ? 1 : Math.signum(virtualServoB.right.times(normalizedHeading));
        double angleCVirtual = Math.acos(virtualServoB.armDirection.times(normalizedHeading)) * biasedSignBVirtual;
        double angleCActual = Math.acos(virtualServoB.forward.times(normalizedHeading)) * biasedSignBActual;

        //rotate and actuate virtual and hardware servo (respectively)
        conversionC.angle2Scalar(3 * Math.PI / 4 - angleBActual);
        virtualServoC.rotateArm(angleCVirtual);
        servoC.setPosition(1-conversionC.getOutput());

        telemetry.addData("Target Vector: ", restrictedTarget);

         */

        //positive is CCW
        double dpForwardA = EULMathEx.safeACOS(virtualServoA.forward.times(targetDir)) * -Math.signum(virtualServoA.right.times(targetDir));
        double locA = EULMathEx.lawOfCosines(virtualServoA.armLength, restrictedTarget.length(), virtualServoB.armLength);
        double armAngleA = dpForwardA + (bottomSolution ? -locA : locA);
        double realAngleA = conversionA.angleToServo(armAngleA);
        //double dpArmA = EULMathEx.safeACOS(virtualServoA.armDirection.times(targetDir)) * -Math.signum(virtualServoA.armDirectionNormal.times(targetDir));
        virtualServoA.setArmAngle(armAngleA);
        targetDir = restrictedTarget.minus(virtualServoB.position).normalized();

        double dpForwardB = EULMathEx.safeACOS(virtualServoB.forward.times(targetDir)) * -Math.signum(virtualServoB.right.times(targetDir));
        double realAngleB = conversionB.angleToServo(dpForwardB);
        //double dpArmB = EULMathEx.safeACOS(virtualServoB.armDirection.times(targetDir)) * -Math.signum(virtualServoB.armDirectionNormal.times(targetDir));
        virtualServoB.rotateArm(dpForwardB);
        //targetDir = restrictedTarget.minus(virtualServoC.position).normalized();

        double dpForwardC = EULMathEx.safeACOS(virtualServoC.forward.times(endHeading)) * -Math.signum(virtualServoC.right.times(endHeading));
        double realAngleC = conversionC.angleToServo(dpForwardC);
        //double dpArmC = EULMathEx.safeACOS(virtualServoC.armDirection.times(endHeading)) * -Math.signum(virtualServoC.armDirectionNormal.times(endHeading));
        virtualServoB.setArmAngle(dpForwardC);

        servoA.setPosition(realAngleA);
        servoB.setPosition(realAngleB);
        servoC.setPosition(realAngleC);

        telemetry.addData("Real Angle A: ", dpForwardA * EULConstants.RAD2DEG);
        telemetry.addData("Real Angle B: ", dpForwardB * EULConstants.RAD2DEG);
        telemetry.addData("Real Angle C: ", dpForwardC * EULConstants.RAD2DEG);
    }
}
