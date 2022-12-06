package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.AngleConversion;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.VirtualServo;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Disabled
@TeleOp(name = "ServoTestCircleFindOpMode", group = "tester")
public class ServoTestCircleFindOpMode extends LoopUtil {

    public ThreeJointArm newPropArm;

    public static ServoFTC servo;
    public static ServoConfig configA, configB, configC;
    public static AngleConversion servoConversionA, servoConversionB, servoConversionC;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;
    public static double commandedPositionMultiplier;
    public Vector2d betterCommandedPosition;
    public char servoLetter = 'A';
    @Override
    public void opInit() {
        switch(servoLetter) {
            case 'A':
                configA = new ServoConfig("A", false, 0, 0.83);
                servo = new ServoFTC(hardwareMap, telemetry, configA);
                break;
            case 'B':
                configB = new ServoConfig("B",true, 0, 1);
                servo = new ServoFTC(hardwareMap, telemetry, configB);
                break;
            case'C':
                configC= new ServoConfig("C",true, 0, 1);
                servo = new ServoFTC(hardwareMap, telemetry, configC);

        }

        servoConversionA = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionB = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionC = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);

        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        enableJoystick = false;
        commandedPosition = 0.0;
        betterCommandedPosition = new Vector2d(20,4);
        commandedPositionMultiplier = 1;

        newPropArm = new ThreeJointArm(
                new VirtualServo(15, new Vector2d(0, -1), new Vector2d(0, 0)),
                servo,
                new AngleConversion[]{servoConversionA, servoConversionB, servoConversionC},
                15,
                17);
    }

    public void handleInput(double deltaTime){
        gamepadHandler.loop();
        if (gamepadHandler.up("D1:LT")){
            enableJoystick = !enableJoystick;
        }

        if (gamepadHandler.up("D1:DPAD_UP")){ //increase outputSpeed by decimalPlace | now wrong comment
            betterCommandedPosition.y += 0.1 * deltaTime;
        }
        if (gamepadHandler.up("D1:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace | now wrong comment
            betterCommandedPosition.y -= 0.1 * deltaTime;
        }

        if (gamepadHandler.up("D1:DPAD_LEFT")){ //increase outputSpeed by decimalPlace | now wrong comment
            betterCommandedPosition.x -= 0.1 * deltaTime;
        }
        if (gamepadHandler.up("D1:DPAD_RIGHT")){ //decrease outputSpeed by decimalPlace | now wrong comment
            betterCommandedPosition.x += 0.1 * deltaTime;
        }
        betterCommandedPosition = betterCommandedPosition.plus((new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y).times(0.005)));
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        newPropArm.circleFind(betterCommandedPosition);

        telemetry.addData("Commanded Multiplier: ", commandedPositionMultiplier);
        telemetry.addData("Commanded Position: ", betterCommandedPosition);
        telemetry.addData("Servo Turn Position: ", servo.getPosition());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }

}
