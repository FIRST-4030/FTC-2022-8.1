package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main.TFODBase;
import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.deprecated.tfodohm.ODMain.CameraLens;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.managers.TaskManager;

public class PowerPlayTFHandler {
    public TFODBase tfodBase;
    public TaskManager stateMachine;

    public double maxSizePX = 1280, widthCone = 10, widthPole = 2.5, hFOV = CameraLens.C270_FOV[0], vFOV = CameraLens.C270_FOV[1];
    public double pipelineTime = 0;

    private double imgWidthHalf, imgHeightHalf, hFovHalf;

    public PowerPlayTFHandler(HardwareMap hardwareMap, String camName){
        tfodBase = new TFODBase(hardwareMap, camName, "junctionOnly.tflite", new String[]{"Junction Top"});
        stateMachine = new TaskManager();
        stateMachine.alwaysRun = () -> {};

        tfodBase.opInit();

        imgWidthHalf = tfodBase.imgWidth / 2d;
        imgHeightHalf = tfodBase.imgHeight / 2d;
    }

    public float findDepth(double absWidth, double bbPx){
        return (float) ((absWidth/(2* Math.sin((hFOV*bbPx)/(2*maxSizePX)))) - (absWidth/2));
    }

    public double findAngleToX(double scalar, double fov){
        return (fov / (Math.abs(scalar) * 2)) * Math.signum(scalar);
    }

    public double findHeight(double angle, double depth){
        return depth * Math.sin(angle);
    }

    public void update(double deltaTime){
        pipelineTime += deltaTime;
    }
}
