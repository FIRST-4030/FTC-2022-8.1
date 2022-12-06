package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

@Disabled
@TeleOp(name = "TensorFlow3 (Blocks to Java)")
public class TensorFlow3 extends LinearOpMode {

    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;

    String cameraOrientation;
    String zone;
    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // TODO: Enter the type for variable named i
        List<Recognition> recognitions;
        double recLength;
        double index;

        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Camera Front"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Template Default: Minimum Confidence=0.7
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.5F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        // Template Default: Magnification=2.5
        // Template Default: Aspect Ratio=16/9
        tfodUltimateGoal.setZoom(1.1, 16 / 12);
        // Define how the camera is oriented
        cameraOrientation = "Vertical";
        // Display the label and index number for the recognition.
        telemetry.addData("Camera Orientation", cameraOrientation);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            // Get a list of recognitions from TFOD.
            recognitions = tfodUltimateGoal.getRecognitions();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (recognitions.size() == 0) {
                    telemetry.addData("Target Zone", "A");
                    zone = "A";
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                }
                telemetry.addData("Target Zone", zone);
                // Display the location of the top left corner
                // of the detection boundary for the recognition
                telemetry.addData("Width, Height Node ", Double.parseDouble(JavaUtil.formatNumber(recognition.getWidth(), 1)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getHeight(), 1)));
                // Display the label and index number for the recognition.
                telemetry.addData("Node ", recognition.toString());
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfodUltimateGoal.deactivate();

        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(double i) {
        // Display node info.
        if (cameraOrientation.equals("Vertical")) {
            if (recognition.getWidth() > 95) {
                zone = "C";
            } else if (recognition.getWidth() < 75) {
                zone = "B";
            } else {
                zone = "Unknown";
            }
        } else if (cameraOrientation.equals("Horizontal")) {
            if (recognition.getHeight() > 95) {
                zone = "C";
            } else if (recognition.getHeight() < 60) {
                zone = "B";
            } else {
                zone = "Unknown";
            }
        } else {
            // Parse rings based upon label
            if (recognition.getLabel().equals("Single")) {
                zone = "B";
            } else if (recognition.getLabel().equals("Quad")) {
                zone = "C";
            } else {
                zone = "Unknown";
            }
        }
    }
}

