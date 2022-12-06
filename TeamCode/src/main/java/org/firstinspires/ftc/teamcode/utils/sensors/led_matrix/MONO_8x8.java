package org.firstinspires.ftc.teamcode.utils.sensors.led_matrix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MONO_8x8 extends BI_8x8 {

    public MONO_8x8(HardwareMap map, Telemetry telemetry, String name) {
        super(map, telemetry, name);
    }

    @Override
    public void write() {
        // Bitshift left by one bc shit's stupid
        byte[] temp = new byte[16];

        for (int i = 0; i < temp.length; i++) {
            temp[i] = matrix.displayBuffer[i];

            // ty stack overflow
            matrix.displayBuffer[i] = (byte) ((matrix.displayBuffer[i] >>> 1) | (matrix.displayBuffer[i] << (32 - 1)));
        }

        matrix.write();
        matrix.displayBuffer = temp;
    }
}
