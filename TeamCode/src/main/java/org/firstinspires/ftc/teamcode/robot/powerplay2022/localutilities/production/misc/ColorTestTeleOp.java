package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

@Autonomous(name = "ColorTele")
public class ColorTestTeleOp extends LoopUtil {
    public static RevColorRange RCR2;
    public static ColorView.CMYcolors SeenColor;
    public static ColorView CV2;
    public static double ColorT1;

    @Override
    public void opInit() {
        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        CV2.update(RCR2.color(), RCR2.distance());
        telemetry.addData("Dist: ", RCR2.distance());
        telemetry.addData("ColorBetter: ", CV2.getColorBetter(80));
        telemetry.addData("Color: ", CV2.getColor());
        telemetry.addData("Color: ", CV2.convertRGBToHSV(CV2.colorInput)[0]);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}

