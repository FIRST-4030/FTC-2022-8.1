package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.depreciated.servos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Disabled
@TeleOp(name = "ArmTest", group = "Tester")
public class ArmTest extends LoopUtil {

    public static ServoFTC servoA, servoB, servoC;
    public static ServoConfig configA, configB, configC;
    public static ServoArmController servoArmController;

    public static InputHandler gamepadHandler;

    public static boolean emergencyStop = false;

    public Vector2d target, cHeading;

    @Override
    public void opInit() {

        gamepadHandler = InputAutoMapper.normal.autoMap(this);

        configA = new ServoConfig("A",false, 0, 0.75);
        configB = new ServoConfig("B",false, 0, 1);
        configC = new ServoConfig("C",false, 0, 1);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);

        servoArmController = new ServoArmController(
                new Vector2d(0, -1), //starting orientation of the servos
                new Vector2d(0, -1), //starting orientation of the servos
                new Vector2d(0, -1), //starting orientation of the servos
                15, //servo AB arm length
                13, //servo BC arm length
                servoA, //reference servoA
                servoB, //reference servoB
                servoC, //reference servoC
                new ServoAngleConversion(0, Math.PI), //top servo is restricted to 180 deg movement
                new ServoAngleConversion(0, (3 * Math.PI) / 2), //[0, 270] degrees is what the unrestricted servo can do
                new ServoAngleConversion(0, (3 * Math.PI) / 2) //[0, 270] degrees is what the unrestricted servo can do
        );

        target = new Vector2d(15, -13);
        cHeading = new Vector2d(0, -1);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        if (!emergencyStop){
            //servoArmController.calculateViaPropagation(target, cHeading);
            servoArmController.calculateByCosines(target, telemetry);
        }else{ double x = 0/0; }
    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        gamepadHandler.loop();
        if (gamepad1.dpad_up){
            target.y += 1 * deltaTime * EULConstants.MS2SEC;
        } else if (gamepad1.dpad_down){
            target.y -= 1 * deltaTime * EULConstants.MS2SEC;
        }

        if (gamepad1.dpad_right){
            target.x += 1 * deltaTime * EULConstants.MS2SEC;
        } else if (gamepad1.dpad_left){
            target.x -= 1 * deltaTime * EULConstants.MS2SEC;
        }

        if (gamepadHandler.up("D1:RT")) emergencyStop = !emergencyStop;

        telemetry.addData("Target: ", target);
    }

    @Override
    public void opStop() {

    }
}
