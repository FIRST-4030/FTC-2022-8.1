package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

public class RingStackTF {

    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;

    private String cameraOrientation;
    private String zone;
    private List<Recognition> recognitions;
    double recLength;
    double index;

    private HardwareMap map;
    Telemetry telemetry;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    public RingStackTF(HardwareMap m, Telemetry telem) {
        telemetry = telem;
        map = m;
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                map.get(WebcamName.class, "Logi"), // cameraName
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
        // Template Default: Minimum Confidence=0.65
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7f, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        // Template Default: Magnification=2.5
        // Template Default: Aspect Ratio=16/9
        tfodUltimateGoal.setZoom(1.5, (16 / 12));
        // Define how the camera is oriented
        cameraOrientation = "Vertical";
        // Display the label and index number for the recognition.
        telemetry.addData("Camera Orientation", cameraOrientation);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }


    public int getTargetZone() {
        // Get a list of recognitions from TFOD.
        recognitions = tfodUltimateGoal.getRecognitions();
        // Put run blocks here.
        // Put loop blocks here.
        recLength = recognitions.size();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
            telemetry.addData("Target Zone", "A");
            zone = "A";
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition : recognitions) {
                // Display info.
                displayInfo(recognition);
                // Increment index.
                index = index + 1;
            }
            telemetry.addData("Target Zone", zone);

        }
        telemetry.update();
        // Deactivate TFOD.
        tfodUltimateGoal.deactivate();

        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();
        switch(zone) {
            case "A":
                return 0;
            case "B":
                return 1;
            case "C":
                return 2;
        }
        return 1;
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(Recognition recognition) {
        // Display node info.
        telemetry.update();
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Width, Height Node ", Double.parseDouble(JavaUtil.formatNumber(recognition.getWidth(), 1)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getHeight(), 1)));
        if (cameraOrientation.equals("Vertical")) {
            if (recognition.getWidth() > 95) {
                zone = "C";
            } else if (recognition.getWidth() < 65) {
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
