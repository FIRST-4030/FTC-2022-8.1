package org.firstinspires.ftc.teamcode.utils.pathfinder.core.drives.sample;

import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.pathfinder.utilities.DriveSpec;
import org.firstinspires.ftc.teamcode.utils.pathfinder.utilities.EncoderTickBuffer;
import org.firstinspires.ftc.teamcode.utils.pathfinder.utilities.MotionMaker;
import org.firstinspires.ftc.teamcode.utils.pathfinder.utilities.PathfinderPath;
import org.firstinspires.ftc.teamcode.utils.pathfinder.utilities.PathfinderPose;

public class PathfinderMecanumMotionMaker extends MotionMaker {

    public DriveSpec driveSpec;

    public PathfinderMecanumMotionMaker bindDrive(DriveSpec spec){
        driveSpec = spec;
        return this;
    }

    protected double calcEllipticalVelocity(Vector2d dir){
        return (driveSpec.advancementTickPerMeasurementUnit * driveSpec.lateralTickPerMeasurementUnit) /
                (Math.sqrt(dir.y * driveSpec.lateralTickPerMeasurementUnit * dir.y * driveSpec.lateralTickPerMeasurementUnit +
                           dir.x * driveSpec.advancementTickPerMeasurementUnit * dir.x * driveSpec.advancementTickPerMeasurementUnit));
    }

    @Override
    public void buildPath(PathfinderPath path, EncoderTickBuffer[] encoderTickBuffers) {

    }

    /**
     *
     * @param currentPose
     * @param targetPose
     * @param type
     * @param rwEncoderBuffers [FL FR BL BR] order
     */
    @Override
    public void process(PathfinderPose currentPose, PathfinderPose targetPose, PathfinderPath.TransitionType type, EncoderTickBuffer[] rwEncoderBuffers) {
        Vector2d translationDirection = currentPose.toRelative(targetPose.pos);

        //translation handling
        float translationLength = (float) translationDirection.length();
        double expectedVelocity;
        double tickCountFL = rwEncoderBuffers[0].rawValues.get(rwEncoderBuffers[0].rawValues.size() - 1), tickCountFR = rwEncoderBuffers[1].rawValues.get(rwEncoderBuffers[1].rawValues.size() - 1), tickCountBL = rwEncoderBuffers[2].rawValues.get(rwEncoderBuffers[2].rawValues.size() - 1), tickCountBR = rwEncoderBuffers[3].rawValues.get(rwEncoderBuffers[3].rawValues.size() - 1);
        double powerFL = 0, powerFR = 0, powerBL = 0, powerBR = 0;

        if (translationLength == 0){
            translationDirection = new Vector2d(0, 1);
        } else {
            translationDirection.normalize();
        }

        expectedVelocity = calcEllipticalVelocity(translationDirection);

        tickCountFL += translationLength / expectedVelocity;
        tickCountFR += translationLength / expectedVelocity;
        tickCountBL += translationLength / expectedVelocity;
        tickCountBR += translationLength / expectedVelocity;

        //rotation handling
        double absTurnRadians = (1 - (currentPose.dir.times(targetPose.dir) + 1) / 2) * EULConstants.PI;
        int turnDir = (int) Math.round(Math.signum(currentPose.normal.times(targetPose.dir)));

        double arcLength = absTurnRadians * driveSpec.expectedTurnRadius * (turnDir == 0 ? 1 : turnDir);

        tickCountFL += arcLength / driveSpec.ticksPerTurn;
        tickCountFR -= arcLength / driveSpec.ticksPerTurn;
        tickCountBL += arcLength / driveSpec.ticksPerTurn;
        tickCountBR -= arcLength / driveSpec.ticksPerTurn;

        //finally pass all values to buffers
        rwEncoderBuffers[0].addRawValue(tickCountFL, powerFL);
        rwEncoderBuffers[1].addRawValue(tickCountFR, powerFR);
        rwEncoderBuffers[2].addRawValue(tickCountBL, powerBL);
        rwEncoderBuffers[3].addRawValue(tickCountBR, powerBR);
    }
}
