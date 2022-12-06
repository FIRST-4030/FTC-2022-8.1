package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.gamepad.PAD_KEY;

import java.util.HashMap;

public class InputAutoMapper {

    public static final InputAutoMapper normal = new InputAutoMapper();

    private HashMap<PAD_KEY, String> padKeyLegend = new HashMap<>();

    /**
     * Creates a custom key mapper able to be configured
     */
    public InputAutoMapper(){
        padKeyLegend.put(PAD_KEY.left_stick_x, "LS_X");
        padKeyLegend.put(PAD_KEY.left_stick_y, "LS_Y");
        padKeyLegend.put(PAD_KEY.right_stick_x, "RS_X");
        padKeyLegend.put(PAD_KEY.right_stick_y, "RS_Y");

        padKeyLegend.put(PAD_KEY.right_bumper, "RB");
        padKeyLegend.put(PAD_KEY.left_bumper, "LB");
        padKeyLegend.put(PAD_KEY.right_trigger, "RT");
        padKeyLegend.put(PAD_KEY.left_trigger, "LT");

        //register dpad
        padKeyLegend.put(PAD_KEY.dpad_up, "DPAD_UP");
        padKeyLegend.put(PAD_KEY.dpad_down, "DPAD_DOWN");
        padKeyLegend.put(PAD_KEY.dpad_left, "DPAD_LEFT");
        padKeyLegend.put(PAD_KEY.dpad_right, "DPAD_RIGHT");

        //register buttons
        padKeyLegend.put(PAD_KEY.a, "A");
        padKeyLegend.put(PAD_KEY.b, "B");
        padKeyLegend.put(PAD_KEY.x, "X");
        padKeyLegend.put(PAD_KEY.y, "Y");

        //register guide and start
        padKeyLegend.put(PAD_KEY.guide, "GUIDE");
        padKeyLegend.put(PAD_KEY.start, "START");
        padKeyLegend.put(PAD_KEY.back, "BACK");
    }

    public InputAutoMapper changePadKeyLegend(PAD_KEY key, String nName){
        if (padKeyLegend.containsKey(key)){
            padKeyLegend.put(key, nName);
        }
        return this;
    }

    public HashMap<PAD_KEY, String> getPadKeyLegend(){
        return padKeyLegend;
    }

    /**
     * This method will map all inputs into the InputHandler and to query the keys, it will always head with "D1:" or "D2:" while the input is in acronym form
     * <br>
     * <br>The default mapping:
     * <br>left_stick_x -> "LS_X"
     * <br>left_stick_y -> "LS_Y"
     * <br>right_stick_x -> "RS_X"
     * <br>right_stick_y -> "RS_Y"
     * <br>right_bumper -> "RB"
     * <br>left_bumper -> "LB"
     * <br>right_trigger -> "RT"
     * <br>left_trigger -> "LT"
     * <br>dpad_up -> "DPAD_UP"
     * <br>dpad_down -> "DPAD_DOWN"
     * <br>dpad_left -> "DPAD_LEFT"
     * <br>dpad_right -> "DPAD_RIGHT"
     * <br>a -> "A"
     * <br>b -> "B"
     * <br>x -> "X"
     * <br>y -> "Y"
     * <br>guide -> "GUIDE"
     * <br>start -> "START"
     * <br>back -> "BACK"
     * @param opMode - OpMode where the object is called
     * @return {@code InputHandler}
     */
    public InputHandler autoMap(OpMode opMode){
        InputHandler output = new InputHandler(opMode);

        GAMEPAD[] drivers = new GAMEPAD[]{GAMEPAD.driver1, GAMEPAD.driver2};
        PAD_KEY[] keys = PAD_KEY.values();

        for (int id = 0; id < 2; id++){
            GAMEPAD driver = drivers[id];
            for (PAD_KEY key: keys) {
                output.register("D" + (id + 1) + ":" + padKeyLegend.get(key), driver, key);
            }
        }

        return output;
    }

    public void logLegend(Telemetry telemetry){
        PAD_KEY[] keys = PAD_KEY.values();
        telemetry.addData(getClass().getSimpleName(), "log BEGIN");
        for (PAD_KEY key: keys) {
            telemetry.addData( key.toString() + " is mapped to: ", padKeyLegend.get(key));
        }
        telemetry.addData(getClass().getSimpleName(), "log END");
    }
}
