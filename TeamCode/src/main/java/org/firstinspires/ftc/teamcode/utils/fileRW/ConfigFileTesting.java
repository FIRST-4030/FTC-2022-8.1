package org.firstinspires.ftc.teamcode.utils.fileRW;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.util.ArrayList;

@Autonomous(name = "FileRW", group = "Test")
@Config
public class ConfigFileTesting extends LoopUtil {
    @Override
    public void opInit() {
        ConfigFileUtil.init();

        ArrayList<Integer> matrix = new ArrayList<>();
        matrix.add(0);
        matrix.add(1);
        matrix.add(2);
        matrix.add(3);

        ConfigFileUtil.writeToConfig("VirusPrototype1", matrix, 2);

        Object[][] output = new Object[2][2];

        ConfigFileUtil.readConfig("VirusPrototype1", output, ConfigFileUtil.ConfigDataType.INT);
        telemetry.addData("Output: ", output[0][0] + "," + output[0][1] + ",\n" + output[1][0] + "," + output[1][1]);

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
