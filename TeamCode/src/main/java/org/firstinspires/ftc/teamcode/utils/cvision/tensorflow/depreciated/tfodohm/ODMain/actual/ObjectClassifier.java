package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain.actual;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.utils.threadingUtils.MRUTask;

import java.util.ArrayList;
import java.util.List;

public class ObjectClassifier extends MRUTask {

    private enum INTERNAL_STATE{
        IDLE,
        VERIFYING,
        SCANNING,
        SHUTDOWN
    }

    private static INTERNAL_STATE state = INTERNAL_STATE.IDLE;

    private static ArrayList<BoundingBox> boundingBoxes;
    private static boolean isVerified = false,
                    isDone = false;

    private static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private AngleUnit angleUnit;

    public ObjectClassifier(HardwareMap hardwareMap, String camName, String tfliteModel, String[] labels, AngleUnit angleUnit){

        this.angleUnit = angleUnit;

        try {
            //initialize Vuforia
            VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
            vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
            vuforiaParameters.cameraName = hardwareMap.get(WebcamName.class, camName);
            vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            vuforiaParameters.useExtendedTracking = false;
            vuforiaParameters.cameraMonitorFeedback = null;
            vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        } catch (Exception e){
            e.printStackTrace();
        }

        try {
            //initialize TensorFlow
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfParameters.minResultConfidence = 0.69f;
            tfParameters.isModelTensorFlow2 = true;
            tfParameters.useObjectTracker = true;
            tfParameters.inputSize = 320;
            tfParameters.maxFrameRate = 30;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfParameters, this.vuforia);
            tfod.loadModelFromAsset(tfliteModel, labels);
        } catch (Exception e){
            e.printStackTrace();
        }

        tfod.activate(); //activates TensorFlow
    }

    @Override
    public void init() {

    }

    @Override
    public void update(double deltaTime) {
        synchronized (state) {
            isDone = false;
            switch (state) {
                case IDLE:
                    //do nothing here
                    break;
                case SHUTDOWN:
                    //dispose resources being used
                    boundingBoxes.clear();
                    tfod.shutdown();
                    vuforia.close();
                    break;
                case VERIFYING:
                    isVerified = tfod.getRecognitions().size() > 0;
                    state = INTERNAL_STATE.IDLE;
                    break;
                case SCANNING:
                    if (isVerified) {
                        boundingBoxes.clear(); //get rid of the old recognitions
                        List<Recognition> recognitions = tfod.getRecognitions();

                        for (int i = 0; i < recognitions.size(); i++) {
                            Recognition cachedRecognition = recognitions.get(i);
                            boundingBoxes.add(
                                    new BoundingBox(cachedRecognition.getTop(),
                                            cachedRecognition.getBottom(),
                                            cachedRecognition.getRight(),
                                            cachedRecognition.getLeft(),
                                            cachedRecognition.estimateAngleToObject(angleUnit),
                                            angleUnit));
                        }
                    }
                    state = INTERNAL_STATE.IDLE;
                    break;
            }
            isDone = true;
        }
    }

    @Override
    public void fixed_update(double deltaTime) {

    }

    public static void queueScan(){
        if (isVerified) {
            waitUntilIdle();
            state = INTERNAL_STATE.SCANNING;
        }
    }

    public static void queueVerify(){
        waitUntilIdle();
        state = INTERNAL_STATE.VERIFYING;
    }

    public static void dispose(){
        state = INTERNAL_STATE.IDLE;
        waitUntilIdle();
    }

    public static void waitUntilIdle(){
        while(!isDone){}
    }

    public static void waitUntilVerified(){
        while(!isVerified){}
    }

    public static boolean isDone(){
        return isDone;
    }

    public static boolean isVerified(){
        return isVerified;
    }
}
