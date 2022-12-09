package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.ext.TFBoundingBox;
import org.firstinspires.ftc.teamcode.utils.general.misc.RunOnce;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class TFODBase{

    public enum STATE{SCANNING, IDLE}

    public boolean isDone = true;
    public boolean isInitialized = false;
    private boolean imgSet = false;

    public float confidenceThreshold = 0.8f;

    public int inputSize = 320;
    public int imgWidth = 1, imgHeight = 1;


    public static final String VUFORIA_KEY = "AXthH7r/////AAABmUBut8RIBEvFrdrtQGl4hdETAg9rhvKSD9a7lhHQzbOJ2Kov3prL1gMb7sjYXb3xJJBicz42tlvrHeBi20f1+vlvbAftlIrNw2W6+f8zcMb+ROLXXv1KIsaS/FL+CZ8NmCDveQTeKHAPWkvnm39dB4TCp7BQZ76edXsacf8S1MnBanG+8M/8HP3yVj1JvQ6yMUGiz2xRfFf53fUJE5Pp9+ePeWDIWgweBnAU+ZaqecvnAywaO0ln5Y/QAF2zAhFJz5d2f350EfeueEbUr7WKxY+EhPUKu0dSth8NJYKpWkre2ybVQVqpF9eqFJLyPi6e2CS1Q0DN362UkGNvyPlYb+J4/74hzzQuSk0sxIFZD5RJ";
    public String tensorflowModel;
    public String cameraName;
    public String[] tensorflowLabels;

    public HashMap<String, ArrayList<Recognition>> recognitions;
    public HashMap<String, ArrayList<TFBoundingBox>> boundingBoxes;

    public VuforiaLocalizer vuforiaLocal;
    public TFObjectDetector tensorflow;
    public HardwareMap hardwareMap;
    public STATE state;

    public TFODBase(HardwareMap hardwareMap, String cameraName, String tensorflowModel, String[] tensorflowLabels){
        this.cameraName = cameraName;
        this.tensorflowModel = tensorflowModel;
        this.tensorflowLabels = tensorflowLabels;
        this.hardwareMap = hardwareMap;
    }

    protected void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, cameraName);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = null;

        vuforiaLocal = ClassFactory.getInstance().createVuforia(parameters);
    }

    protected void initTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = confidenceThreshold;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.useObjectTracker = true;
        tfodParameters.inputSize = inputSize;
        tensorflow = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocal);
        tensorflow.loadModelFromAsset(tensorflowModel, tensorflowLabels);
    }

    public void opInit() {
        initVuforia();
        initTensorFlow();
        tensorflow.activate();
        isInitialized = true;
    }

    public void scan(){
        state = STATE.SCANNING;
        isDone = false;

        if (tensorflow == null) return; //abort if tensorflow is null

        List<Recognition> tfOutput = tensorflow.getRecognitions();


        if (recognitions == null) return; //abort if tensorflow doesn't recognize anything; also persists the current recognitions in cache
        clearRecognitions(); //clear to prep for refilling

        for (Recognition output: tfOutput) {
            if (!checkLabels(output.getLabel())) {
                this.recognitions.put(output.getLabel(), new ArrayList<>());
                this.boundingBoxes.put(output.getLabel(), new ArrayList<>());

                if (!imgSet) {
                    imgWidth = output.getImageWidth();
                    imgHeight = output.getImageHeight();
                    imgSet = true;
                }
            }

            this.recognitions.get(output.getLabel()).add(output); //appends recognition to properly labeled arraylist
            this.boundingBoxes.get(output.getLabel()).add(new TFBoundingBox(output)); //also appends to the bounding box arraylist
        }

        state = STATE.IDLE;
        isDone = true;
    }

    public void clearRecognitions(){
        for (String label: tensorflowLabels) {
            this.recognitions.put(label, new ArrayList<>());
            this.boundingBoxes.put(label, new ArrayList<>());
        }
    }

    public boolean checkLabels(String in){
        for (String s: tensorflowLabels) {
            if (s.equals(in)){
                return true;
            }
        }

        return false;
    }

    public synchronized void waitForIdle(){
        while (state != STATE.IDLE);
    }

    public synchronized void waitForScanning(){
        while (state != STATE.SCANNING);
    }

    public void opStop() {
        tensorflow.shutdown();
    }
}
