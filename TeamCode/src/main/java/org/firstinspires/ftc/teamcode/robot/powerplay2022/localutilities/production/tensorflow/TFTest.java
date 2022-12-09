package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.base.main.TFODBase;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "TF TEST")
public class TFTest extends LoopUtil {

    public static TFODBase tfodBase;

    @Override
    public void opInit() {
        tfodBase = new TFODBase(hardwareMap, "Webcam 1", "PowerPlay.tflite", new String[]{
                "1 Bolt",
                "2 Bulb",
                "3 Panel"
        });

        tfodBase.opInit();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
