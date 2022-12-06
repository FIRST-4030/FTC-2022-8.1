package org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.general.maths.piecewise.PiecewiseFunction;

@Config
@Disabled
@Autonomous(name = "CodeTest", group = "Test")
public class PureCodeTest extends OpMode {
    // Config
    public static boolean DEBUG = false;
    public static double TICKS_PER_INCH = 43.24;
    public static double trackWidth = 15.25;
    public static double trackWidthHalf = trackWidth / 2.0;

    // Members
    private boolean enabled = false;
    final private ElapsedTime rampTimer = new ElapsedTime();
    private boolean started;
    private boolean done;
    final private PiecewiseFunction speedCurve = new PiecewiseFunction();
    final private PiecewiseFunction speedCurveL = new PiecewiseFunction();
    final private PiecewiseFunction speedCurveR = new PiecewiseFunction();
    private int counts = 0;

    private double driveLeftPower;
    private double driveRightPower;
    private double driveLeftPosition;
    private double driveRightPosition;

    // simulation variables
    final private ElapsedTime simTimerElapsed = new ElapsedTime();
    final private ElapsedTime simTimerL = new ElapsedTime();
    final private ElapsedTime simTimerR = new ElapsedTime();
    private boolean simulationStarted = false;
    private final double simulationMaxSpeed = 40.0 * TICKS_PER_INCH; // ticks per second at max speed (power of 1.0)
    private double simulationLastTime = 0.0;
    private double simulationLeftOldPosition, simulationRightOldPosition;

    // Standard methods
    @Override
    public void init() {
        try {
            done = false;
            started = false;
            enabled = true;
            counts = 0;

            simulationStarted = false;
            simTimerL.reset();
            simTimerR.reset();
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
        done = false;
        started = false;

        counts++;
        RobotLog.clearGlobalErrorMsg();
        RobotLog.clearGlobalWarningMsg();
        RobotLog.d(",try number," + counts);
        PiecewiseFunction testMe = new PiecewiseFunction();

        testMe.setDefaultValue(40.0);
        testMe.debug = false;
        testMe.addElement(-1,0);
        testMe.addElement(-0.91304347826087,0.407862239984646);
        testMe.addElement(-0.826086956521739,0.56354266942677);
        testMe.addElement(-0.739130434782609,0.673562321079551);
        testMe.addElement(-0.652173913043478,0.758069381485335);
        testMe.addElement(-0.565217391304348,0.824941998304795);
        testMe.addElement(-0.478260869565217,0.878217820727137);
        testMe.addElement(-0.391304347826087,0.920261325587684);
        testMe.addElement(-0.304347826086957,0.952560969574202);
        testMe.addElement(-0.217391304347826,0.976084535680159);
        testMe.addElement(-0.130434782608696,0.991456891390555);
        testMe.addElement(-0.0434782608695652,0.999054373310961);
        testMe.addElement(0.0434782608695652,0.999054373310961);
        testMe.addElement(0.130434782608696,0.991456891390555);
        testMe.addElement(0.217391304347826,0.976084535680159);
        testMe.addElement(0.304347826086957,0.952560969574202);
        testMe.addElement(0.391304347826087,0.920261325587684);
        testMe.addElement(0.478260869565217,0.878217820727137);
        testMe.addElement(0.565217391304348,0.824941998304795);
        testMe.addElement(0.652173913043478,0.758069381485335);
        testMe.addElement(0.739130434782609,0.673562321079551);
        testMe.addElement(0.826086956521739,0.563542669426771);
        testMe.addElement(0.91304347826087,0.407862239984646);
        testMe.addElement(1,0);

        testMe.setClampLimits(true);
        testPiecewise(testMe, "Clamped", 0.125, true);
        testMe.setClampLimits(false);
        testPiecewise(testMe, "Not Clamped", 0.125, false);

        testMe.reset();
        testMe.setClampLimits(false);
        testMe.debug = false;
        testMe.setDefaultValue(40.0);
        testMe.addElement(0,0);
        testMe.addElement(1,0);
        testMe.addElement(1,1);
        testMe.addElement(2,1);
        testMe.addElement(2,0);
        testMe.addElement(3,0);
        testMe.addElement(3,1);

        testMe.setDefaultHigh(true);
        testPiecewise(testMe, "Default High", 0.125, true);

        testMe.setDefaultHigh(false);
        testPiecewise(testMe, "Default Low", 0.125, false);

        testMe.reset();
        testMe.setClampLimits(true);
        testMe.debug = false;
        testMe.setDefaultValue(40.0);
        testMe.addElement(0,0);
        testMe.addElement(1,1);

        double X = testMe.getFirstX(), Y = testMe.getFirstY();
        while (!testMe.isClamped()) {
            Y = testMe.getY(X);
            X = X + 0.125;
        }
        RobotLog.d(",Final Y," + Y);

        testPiecewise(testMe, "While Loop", 0.125, true);

        testMe.reset();
        testMe.addElement(12,3);
        testMe.addElement(11,-3);
        testMe.addElement(10,2.5);
        testMe.addElement(9,-2.5);
        testMe.addElement(8,2);
        testMe.addElement(7,-2);
        testMe.addElement(6,1.5);
        testMe.addElement(5,-1.5);
        testMe.addElement(4,1);
        testMe.addElement(3,-1);
        testMe.addElement(2,0.5);
        testMe.addElement(1,-0.5);
        testMe.addElement(0,0);

        testPiecewise(testMe, "Jagged Function", 0.5, true);

        telemetry.addData("logfile name", RobotLog.getLogFilename());
        enabled = true;
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        if (!done) {
//            arcToDistance(35.625, 15, 0.2, 0.4);
//            arcTo(-50, 40, 0.2, 0.4);
            driveTo(-0.2, -0.4, -52);
        }

        simulateMovement();
    }

    /**
     * Initialize the headers for all of the logged data
     * Note that this does NOT add headers for any function-specific data
     */
    private void logDataInit() {
        RobotLog.d("");
        RobotLog.d(",Time (s),Function,Left Position (in),Right Position (in),Left Velocity (in/s),Right Velocity (in/s)");
    }

    /**
     * Log the basic drive data to a text file. Also log the "additionalData" string;
     */
    private void logData(String functionName, String additionalData) {
        RobotLog.d("," + functionName + "," + getRuntime() + "," +
                driveLeftPosition / TICKS_PER_INCH + "," + driveRightPosition / TICKS_PER_INCH + "," +
                driveLeftPower * 40.0 + "," + driveLeftPower * 40.0 + "," +
                additionalData);
    }

    private void simulateMovement() {
        if (!simulationStarted) {
            simTimerL.reset();
            simTimerR.reset();
            driveLeftPosition = 0.0;
            driveRightPosition = 0.0;
            simTimerElapsed.reset();
            simulationLastTime = 0.0;
            RobotLog.d("");
            RobotLog.d(",Navigation");
            RobotLog.d(",Time (s),Left Position (in),Right Position (in),Left Velocity (in/s),Right Velocity (in/s),Radius (in),Angle (deg),done,isClamped");
        }

        double timeDelta = simTimerElapsed.seconds() - simulationLastTime;
        driveLeftPosition += Math.max(Math.min(driveLeftPower, 1), -1) * simulationMaxSpeed * timeDelta;
        driveRightPosition += Math.max(Math.min(driveRightPower, 1), -1) * simulationMaxSpeed * timeDelta;

        double leftPosition = driveLeftPosition / TICKS_PER_INCH;
        double rightPosition = driveRightPosition / TICKS_PER_INCH;
        double leftVelocity = (leftPosition - simulationLeftOldPosition) / timeDelta;
        double rightVelocity = (rightPosition - simulationRightOldPosition) / timeDelta;
        double radiusOfCurvature = trackWidth * ( driveLeftPosition + driveRightPosition ) / ( 2 * Math.abs(driveLeftPosition-driveRightPosition));
        double angle = Math.toDegrees( Math.min(driveLeftPosition,driveRightPosition)/(radiusOfCurvature-trackWidth/2));

        telemetry.addData("left position:", leftPosition);
        telemetry.addData("right position:", rightPosition);
        RobotLog.d("," + simTimerElapsed.seconds() + "," +
                leftPosition + "," + rightPosition + "," +
                (simulationStarted ? leftVelocity : "") + "," + (simulationStarted ? rightVelocity : "") + "," +
                radiusOfCurvature + "," + angle + "," + done + "," + speedCurve.isClamped());

        simulationStarted = true;
        simulationLeftOldPosition = leftPosition;
        simulationRightOldPosition = rightPosition;
        simulationLastTime = simTimerElapsed.seconds();
    }

    @Override
    public void stop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Stop the drive motors
        driveLeftPower = 0;
        driveRightPower = 0;
    }

    // Custom methods
    public boolean isBusy() {
        return (driveLeftPower != 0 || driveRightPower != 0);
    }

    public boolean isDone() {
        return done;
    }

    public void driveTo(double speedMin, double speedMax, double distance) {
        if (distance == 0) {
            return;
        }
        double midTicks = distance * TICKS_PER_INCH;

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeftPower = (speedCurve.getY(driveLeftPosition * 1.0));
            driveRightPower = (speedCurve.getY(driveRightPosition * 1.0));
            done = speedCurve.isClamped();
        }

        if (!started) {
            // This resets the encoder ticks to zero on both motors
            driveLeftPosition = 0;
            driveRightPosition = 0;
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurve.setClampLimits(true);
            speedCurve.addElement(0.00 * midTicks, speedMin);
            speedCurve.addElement(0.125 * midTicks, speedMax);
            speedCurve.addElement(0.625 * midTicks, speedMax);
            speedCurve.addElement(1.00 * midTicks, speedMin);
            started = true;
            done = false;
        } else if (done) {
            driveLeftPower = (0);
            driveRightPower = (0);
            started = false;
            speedCurve.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
        telemetry.addData("left ticks", driveLeftPosition);
        telemetry.addData("right ticks", driveRightPosition);
        telemetry.addData("leftVel", driveLeftPower);
        telemetry.addData("rightVel", driveRightPower);
        logData("driveTo()", started + "," + done + "," + speedCurve.isClamped() + "," + speedCurve.isValid() + "," + speedCurve.getSize());
    }

    // angle = angle of rotation, degrees; positive is left, negative is right
    // speedMin - minimum speed of the drive, 0 - 1
    // speedMax - maximum speed of the drive, 0 - 1
    public void turnTo(double angle, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }

        speedMin = Math.abs(speedMin);
        speedMax = Math.abs(speedMax);
        double speed = 0;
        double v = 40 * (speedMax * 4 + speedMin * 2) / 6; // inches per second
        double arcLength = Math.PI * (angle / 180.0) * trackWidthHalf; // inches

        // Don't allow new moves if we're still busy
        double time = Math.abs(arcLength / v);
        double leftVel;
        double rightVel;

        if (angle < 0) {
            leftVel = v;
            rightVel = -v;
        } else {
            leftVel = -v;
            rightVel = v;
        }

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            driveLeftPower = speed * (leftVel / v);
            driveRightPower = speed * (rightVel / v);
        }

        if (!started) {
            rampTimer.reset();
            driveLeftPower = speedMin * (leftVel / v);
            driveRightPower = speedMin * (rightVel / v);

            // initialize speedCurve to have time be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurve.setClampLimits(true);
            speedCurve.addElement(0.00 * time, speedMin);
            speedCurve.addElement(0.25 * time, speedMax);
            speedCurve.addElement(0.75 * time, speedMax);
            speedCurve.addElement( time, speedMin);

            started = true;
            done = false;
        } else if (done) {
            driveLeftPower = 0;
            driveRightPower = 0;
            started = false;
            speedCurve.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }


    // angle = angle of rotation, degrees; positive is left, negative is right
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    public void arcTo(double angle, double r, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }

        // it should be, but ensure that the radius is positive
        r = Math.abs(r);
        if (r < 5) r = 5.0;

        double speed = 0;
        double v; // inches per second
        if (!speedCurve.isValid()) v = 40 * speedMax;
        else v = 40.0 * speedCurve.getAverage();
        double arcLength = Math.PI * (angle / 180.0) * r; // inches
        double arcLengthL;
        double arcLengthR;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        double time = Math.abs(arcLength / v);
        double leftVel = v * arcLengthL / arcLength;
        double rightVel = v * arcLengthR / arcLength;
        double maxRatio = 1;

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds() / time);
            done = speedCurve.isClamped();

            maxRatio = Math.max(Math.abs(leftVel), Math.abs(rightVel)) / Math.abs(v);
            driveLeftPower = speed * (leftVel / v) / maxRatio;
            driveRightPower = speed * (rightVel / v) / maxRatio;
        }

        if (!started) {
            rampTimer.reset();
            driveLeftPower = speedMin * (leftVel / v);
            driveRightPower = speedMin * (rightVel / v);

            // initialize speedCurve to have time be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurve.setClampLimits(true);
            speedCurve.addElement(0.00, speedMin);
            speedCurve.addElement(0.25, speedMax);
            speedCurve.addElement(0.75, speedMax);
            speedCurve.addElement(1.00, speedMin);

            started = true;
            done = false;
        } else if (done) {
            driveLeftPower = 0;
            driveRightPower = 0;
            started = false;
            speedCurve.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        if (isBusy() || !done) telemetry.addData("max ratio", maxRatio);
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    // angle = angle of rotation, degrees; positive is left, negative is right
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    public void arcToTicks(double angle, double r, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }
        double arcLengthL;
        double arcLengthR;
        double leftTicks;
        double rightTicks;
        // it should be, but ensure that the radius is positive
        r = Math.abs(r);

        if (r < 5) r = 5.0;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        leftTicks = arcLengthL * TICKS_PER_INCH;
        rightTicks = arcLengthR * TICKS_PER_INCH;

        double maxRatio = Math.max(leftTicks / rightTicks, leftTicks / rightTicks);
        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeftPower = ((leftTicks / rightTicks) / maxRatio * speedCurveL.getY(driveLeftPosition));
            driveRightPower = ((rightTicks / leftTicks) / maxRatio * speedCurveR.getY(driveRightPosition));
            done = speedCurveL.isClamped() || speedCurveR.isClamped();
        }

        if (!started) {
            // This resets the encoder ticks to zero on both motors
            driveLeftPosition = 0;
            driveRightPosition = 0;
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(0.00 * leftTicks, speedMin);
            speedCurveL.addElement(0.25 * leftTicks, speedMax);
            speedCurveL.addElement(0.75 * leftTicks, speedMax);
            speedCurveL.addElement( leftTicks, speedMin);
            speedCurveR.addElement(0.00 * rightTicks, speedMin);
            speedCurveR.addElement(0.25 * rightTicks, speedMax);
            speedCurveR.addElement(0.75 * rightTicks, speedMax);
            speedCurveR.addElement( rightTicks, speedMin);

            started = true;
            done = false;
        } else if (done) {
            driveLeftPower = 0;
            driveRightPower = 0;
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcToTicks(): Motors in use");
        telemetry.addData("left ticks", driveLeftPosition);
        telemetry.addData("right ticks", driveRightPosition);
        telemetry.addData("left ticks", speedCurveL.getElementX(speedCurveL.getSize() - 1));
        telemetry.addData("right ticks", speedCurveR.getElementX(speedCurveR.getSize() - 1));
        telemetry.addData("leftVel", driveLeftPower);
        telemetry.addData("rightVel", driveRightPower);
    }

    public void arcToOG(double angle, double r, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }

        // it should be, but ensure that the radius is positive
        r = Math.abs(r);
        if (r < 5) r = 5.0;
        double speed = 0;
        double rampTime;
        double v = 40 * (((speedMin + speedMax) / 2 * 0.5) + (speedMax * 0.5)); // inches per second
        double arcLength = Math.PI * (Math.abs(angle) / 180.0) * r; // inches
        double arcLengthL;
        double arcLengthR;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        double time = Math.abs(arcLength / v);
        double leftVel = v * arcLengthL / arcLength;
        double rightVel = v * arcLengthR / arcLength;
        double maxRatio = Math.max(Math.abs(leftVel), Math.abs(rightVel)) / Math.abs(v);

        if (!started) {
            rampTimer.reset();
            driveLeftPower = speedMin * (leftVel / v) / maxRatio;
            driveRightPower = speedMin * (rightVel / v) / maxRatio;
            started = true;
            done = false;
        }

        if ((isBusy() || !done) && started) {
            if (rampTimer.seconds() <= (time * 0.125)) {
                rampTime = time * 0.125;
                speed = (speedMin + (rampTimer.seconds() / rampTime) * (speedMax - speedMin));
            } else if (rampTimer.seconds() <= (time * 0.625)) {
                speed = speedMax;
            } else if (rampTimer.seconds() < time){
                rampTime = time * 0.625;
                speed = (speedMax + (rampTimer.seconds() / rampTime) * (speedMin - speedMax));
            } else {
                speed = 0;
                done = true;
            }
            driveLeftPower = speed * (leftVel / v) / maxRatio;
            driveRightPower = speed * (rightVel / v) / maxRatio;
            telemetry.log().add(getClass().getSimpleName() + "::arcToOG(): Motors in use");
        }

        if (done) {
            driveLeftPower = 0;
            driveRightPower = 0;
            started = false;
        }

        if (isBusy() || !done) telemetry.addData("max ratio", maxRatio);
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    public void arcToDistance(double r, double arcLength, double speedMin, double speedMax) {

        // it should be, but ensure that the radius is positive
        r = Math.abs(r);
        if (r < 5) r = 5.0;
        double speedL;
        double speedR;
        double angle = (arcLength * 180.0) / (Math.PI * r);
        double arcLengthL;
        double arcLengthR;
        if (r > 1000) {
            arcLengthL = arcLength;
            arcLengthR = arcLength;
        } else if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        double leftTicks = arcLengthL * TICKS_PER_INCH;
        double rightTicks = arcLengthR * TICKS_PER_INCH;
        double maxRatio = Math.max(leftTicks / rightTicks, leftTicks / rightTicks);

        if (!started) {
            driveLeftPosition = 0;
            driveRightPosition = 0;
            driveLeftPower = speedMin * (leftTicks / rightTicks) / maxRatio;
            driveRightPower = speedMin * (rightTicks / leftTicks) / maxRatio;
            started = true;
            done = false;
        }

        if ((isBusy() || !done) && started) {
            if (driveLeftPosition <= (leftTicks * 0.125) || driveRightPosition <= (rightTicks * 0.125)) {
                speedL = (speedMin + (driveLeftPosition / leftTicks) * (speedMax - speedMin));
                speedR = (speedMin + (driveRightPosition / rightTicks) * (speedMax - speedMin));
            } else if (driveLeftPosition <= (leftTicks * 0.625) || driveRightPosition <= (rightTicks * 0.625)) {
                speedL = speedMax;
                speedR = speedMax;
            } else if (driveLeftPosition < leftTicks || driveRightPosition < rightTicks) {
                speedL = (speedMax + (driveLeftPosition / leftTicks) * (speedMin - speedMax));
                speedR = (speedMax + (driveRightPosition / rightTicks) * (speedMin - speedMax));
            } else {
                speedL = 0;
                speedR = 0;
                done = true;
            }
            driveLeftPower = speedL * (leftTicks / rightTicks) / maxRatio;
            driveRightPower = speedR * (rightTicks / leftTicks) / maxRatio;
            telemetry.log().add(getClass().getSimpleName() + "::arcToDistance(): Motors in use");
        }
    }

    public void testPiecewise(PiecewiseFunction function, String functionName, Double stepSize, Boolean logElements) {
        RobotLog.d(",");
        RobotLog.d(",name," + functionName);
        if (logElements) {
            RobotLog.d(",index,x,y");
            for (int i = 0; i < function.getSize(); i++) {
                RobotLog.d("," + i + "," + function.getElementX(i) + "," + function.getElementY(i));
            }
        }
        RobotLog.d(",,x,y,isClamped");
        for (Double x = function.getFirstX() - stepSize; x <= function.getLastX() + stepSize; x += stepSize) {
            RobotLog.d(",," + x.toString() + "," + function.getY(x) + "," + function.isClamped());
        }
        RobotLog.d(",element size," + function.getSize());
        RobotLog.d(",clamp limits," + function.getClampLimits());
        RobotLog.d(",default high," + function.getDefaultHigh());
        RobotLog.d(",average," + function.getAverage());
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        driveLeftPower = 0;

        driveRightPower = 0;
    }
}
