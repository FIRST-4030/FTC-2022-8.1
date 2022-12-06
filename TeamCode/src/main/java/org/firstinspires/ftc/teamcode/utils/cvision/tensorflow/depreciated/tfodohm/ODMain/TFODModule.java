package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4fBuilder;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@Autonomous(name = "TFODModuleTest", group = "Test")
public class TFODModule extends OpMode {

    //Vuforia stuff for camera handling
    private static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";
    private VuforiaLocalizer vuforia;
    private static final String CAM_NAME = "MainCam";
    private int imgWidth, imgHeight;

    //init TF stuff for AI object detection
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private TFObjectDetector tfod;
    private List<Recognition> tfodRecognitions = new ArrayList<>();
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //Object detection stuff
    private boolean busy = false;
    private boolean debug = true;
    private ArrayList<DepreciatedVector2F> bbCenterCubeBall = new ArrayList<>();
    private ArrayList<DepreciatedVector2F> bbCenterDuck = new ArrayList<>();
    private ArrayList<DepreciatedVector2F> bbCenterMarker = new ArrayList<>();
    private DepreciatedVector2F error_vector = new DepreciatedVector2F(217456,202302);
    private int cLSCubeBall = 0, cLSDuck = 0, cLSMarker = 0, tLS = 0;
    private String telemetryStringCache = "";
    private CameraLens camera = new CameraLens(CameraLens.C270_FOV);

    /**
     * After calling this class, init() after setting variables
     */
    public TFODModule(){
        //default these values
        camera.setTranslation(new DepreciatedVector3F(0, 0, 0));
        camera.setRotation(new DepreciatedMatrix4f());
    }

    /**
     * After calling this class, init() after setting variables
     */
    public TFODModule(DepreciatedVector3F camPosition, DepreciatedMatrix4f camRotation){
        camera.setTranslation(camPosition);
        camera.setRotation(camRotation);
    }

    /**
     * Initializes TF for us to use
     * <p>This method inits: minResultConfidence; ModelTensorFlow2; inputSize;
     */
    public void initTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfParameters.minResultConfidence = 0.69f;
        tfParameters.isModelTensorFlow2 = true;
        tfParameters.useObjectTracker = true;
        tfParameters.inputSize = 320;
        tfParameters.maxFrameRate = 30;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfParameters, this.vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Initializes Vuforia for us to use
     * <p>This method gives the localizer the VUFORIA_KEY;
     */
    public void initVuforia(){
         VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = hardwareMap.get(WebcamName.class, TFODModule.CAM_NAME);
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaParameters.useExtendedTracking = false;
        vuforiaParameters.cameraMonitorFeedback = null;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
    }

    /**
     * This method force updates the variables
     */
    public void updateVar(){
        Size c = vuforia.getCameraCalibration().getSize();
        imgWidth = c.getWidth();
        imgHeight = c.getHeight();
        cLSCubeBall = bbCenterCubeBall.size();
        cLSDuck = bbCenterDuck.size();
        cLSMarker = bbCenterMarker.size();
    }

    /**
     * This method verifies that an object is in the img so we prevent null pointers
     * @return true = object(s) are in view; false = object(s) not in view
     */
    public boolean verifyImg(){
        if (tfod != null) {
            List<Recognition> temp = tfod.getRecognitions();
            if (temp.size() > 0) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    /**
     * Gets the camera feed, use TF to recognize, and update variables
     */
    public void scan(){
        busy = true;
        String currentLabel;
        DepreciatedVector2F center = new DepreciatedVector2F(), tLeft = new DepreciatedVector2F(), bRight = new DepreciatedVector2F();

        if (tfod != null) {
            bbCenterCubeBall.clear(); //clear
            bbCenterDuck.clear();
            bbCenterMarker.clear();

            tfodRecognitions = tfod.getRecognitions();
            tLS = tfodRecognitions.size();
            for (Recognition recognition : tfodRecognitions) {
                currentLabel = recognition.getLabel().toUpperCase();

                tLeft.setX(recognition.getLeft());
                tLeft.setY(recognition.getTop());

                bRight.setX(recognition.getRight());
                bRight.setY(recognition.getBottom());

                center.setX(((tLeft.getX() + bRight.getX()) / 2) / (0.5f * imgWidth) - 1);
                center.setY(((tLeft.getY() + bRight.getY()) / 2) / (0.5f * imgHeight) - 1);

                switch (currentLabel) {
                    case "BALL":
                    case "CUBE":
                        bbCenterCubeBall.add(center);
                        break;
                    case "DUCK":
                        bbCenterDuck.add(center);
                        break;
                    case "MARKER":
                        bbCenterMarker.add(center);
                        break;
                    default:
                        break;
                }
            }
        }

        busy = false;
    }

    /**
     * Sort for Cubes & Balls (the closest to the center)
     * @return Vector2f bbCoordinate
     */
    public DepreciatedVector2F sortCBBB(){
        if (tfod != null) {
            busy = true;
            int storedIndex = 0;
            float currentLen;
            float cachedLen = 100;

            if (cLSCubeBall > 0) {

                for (int i = 0; i < cLSCubeBall; i++) {

                    currentLen = bbCenterCubeBall.get(i).length();

                    if (currentLen <= cachedLen) {
                        cachedLen = currentLen;
                        storedIndex = i;
                    }
                }
                busy = false;
                return bbCenterCubeBall.get(storedIndex);
            }
            busy = false;
            return error_vector;
        }
        return error_vector;
    }

    /**
     * Sort for Ducks (the closest to the center)
     * @return Vector2f bbCoordinate
     */
    public DepreciatedVector2F sortDBB(){
        if (tfod != null) {
            busy = true;
            int storedIndex = 0;
            float currentLen;
            float cachedLen = 100;

            if (cLSDuck > 0) {

                for (int i = 0; i < cLSDuck; i++) {

                    currentLen = bbCenterDuck.get(i).length();

                    if (currentLen <= cachedLen) {
                        cachedLen = currentLen;
                        storedIndex = i;
                    }
                }
                busy = false;
                return bbCenterDuck.get(storedIndex);
            }
            busy = false;
            return error_vector;
        }
        return error_vector;
    }

    /**
     * Sort for Markers (the closest to the center)
     * @return Vector2f bbCoordinate
     */
    public DepreciatedVector2F sortMBB(){
        if (tfod != null) {
            busy = true;
            int storedIndex = 0;
            float currentLen;
            float cachedLen = 100;

            if (cLSMarker > 0) {

                for (int i = 0; i < cLSMarker; i++) {

                    currentLen = bbCenterMarker.get(i).length();

                    if (currentLen <= cachedLen) {
                        cachedLen = currentLen;
                        storedIndex = i;
                    }
                }
                busy = false;
                return bbCenterMarker.get(storedIndex);
            }
            busy = false;
            return error_vector;
        }
        return error_vector;
    }


    /**
     * When {@param state} is 1, 2, or 3, it corresponds with the following:
     * <br>sort [Cubes & Balls, Ducks, Markers]
     * @param state
     * @return
     */
    public DepreciatedVector2F sortBB(int state){
        if (tfod != null) {
            busy = true;
            int cLS;
            ArrayList<DepreciatedVector2F> bbCenterList;

            switch (state) {
                default:
                case 1:
                    cLS = cLSCubeBall;
                    bbCenterList = bbCenterCubeBall;
                    break;
                case 2:
                    cLS = cLSDuck;
                    bbCenterList = bbCenterDuck;
                    break;
                case 3:
                    cLS = cLSMarker;
                    bbCenterList = bbCenterMarker;
                    break;
            }

            int storedIndex = 0;
            float cachedLen = 100;
            float currentLen;

            if (cLS > 0) {

                for (int i = 0; i < cLS; i++) {

                    currentLen = bbCenterList.get(i).length();

                    if (currentLen <= cachedLen) {
                        cachedLen = currentLen;
                        storedIndex = i;
                    }
                }
                busy = false;
                return bbCenterList.get(storedIndex);
            }
            busy = false;
            return error_vector;
        }
        return error_vector;
    }

    /**
     * Wrapper Method to calculate the given bounding box coordinate into local robot space
     * @param bbCoordinate
     * @return
     */
    public DepreciatedVector3F calcCoordinate(DepreciatedVector2F bbCoordinate){
        if (bbCoordinate != error_vector) {
            return camera.findImgToLocal(bbCoordinate);
        }
        return new DepreciatedVector3F(0,0,0);
    }

    /**
     * Find the busy state of the class; almost like DcMotor.isBusy()
     * @return
     */
    public boolean isBusy(){
        return busy;
    }

    public TFObjectDetector getTfod(){
        return tfod;
    }

    public VuforiaLocalizer getVuforia(){
        return vuforia;
    }

    public String getData(){
        return telemetryStringCache;
    }

    public void logData(){
        if (debug){
            telemetryStringCache = "Debug: ON";
            telemetryStringCache += "\nTotal Objects Recognized: " + (cLSCubeBall + cLSDuck + cLSMarker);
            telemetryStringCache += "\nOR Breakdown: \n" + ("\tCubes & Balls: " + cLSCubeBall + "\n") + ("\tDucks: " + cLSDuck + "\n") + ("\tMarkers: " + cLSMarker + "\n");
            telemetryStringCache += "\nCube & Ball List: \n" + bbCenterCubeBall.toString();
            telemetryStringCache += "\nMarker List: \n" + bbCenterMarker.toString();
            telemetryStringCache += "\nDuck List: \n" + bbCenterDuck.toString();
            telemetryStringCache += "\nCamera imgToLocal: " + camera.getImgToLocal().toString();
        } else {
            telemetryStringCache = "Debug: OFF";
        }
        telemetry.addData("TFODModule Debugging: \n", telemetryStringCache);
    }

    public void initTFStuff(){
        try{
            initVuforia();
        } catch (Exception e) {
            telemetry.log().add("Vuforia Cannot Initialize!");
        }

        try{
            initTensorFlow();
        } catch (Exception e) {
            telemetry.log().add("TensorFlow Cannot Initialize!");
        }

        if (tfod != null){
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        DepreciatedMatrix4f lensRot = DepreciatedMatrix4f.matMul(DepreciatedMatrix4f.matMul(DepreciatedMatrix4fBuilder.buildRotY(-188) , DepreciatedMatrix4fBuilder.buildRotX(-50)), DepreciatedMatrix4fBuilder.buildRotZ(180));

        camera.setTranslation(new DepreciatedVector3F(4.1f, 16.2f, -7.2f));
        camera.setRotation(lensRot);

        telemetry.log().add("Vuforia Class Null? "+ vuforia == null ? "Yes" : "No");
        telemetry.log().add("TF Class Null? "+ tfod == null ? "Yes" : "No");
    }

    @Override
    public void init() {
        try{
            initVuforia();
        } catch (Exception e) {
            telemetry.log().add("Vuforia Cannot Initialize!");
        }

        try{
            initTensorFlow();
        } catch (Exception e) {
            telemetry.log().add("TensorFlow Cannot Initialize!");
        }

        if (tfod != null){
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
        
        telemetry.log().add("Vuforia Class Null? "+ vuforia == null ? "Yes" : "No");
        telemetry.log().add("TF Class Null? "+ tfod == null ? "Yes" : "No");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        updateVar();
        scan();
        logData();
    }

    @Override
    public void stop() {
        tfod.shutdown();
        vuforia.close();
    }


}
