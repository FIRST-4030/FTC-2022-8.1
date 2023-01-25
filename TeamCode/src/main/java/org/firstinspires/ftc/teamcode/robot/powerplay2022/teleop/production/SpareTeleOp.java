package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AngleOffsetHandler;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.AngleConversion;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.VirtualServo;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.tensorflow.TFPipeline;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.io.FileNotFoundException;

//Control mapping

/**
 * DRIVER 1:
 * Left joystick is the drive (forward, backward, strafe) and right joystick is turn angle
 * DRIVER 2:
 * Left joystick controls the servo arm (y for inward and outward, x for turning)
 * Right joystick y controls the vertical portion of the servo arm
 * Claw of servo arm closes on Right Bumper
 */
@TeleOp(name = "SpareTeleOp", group = "actual")
public class SpareTeleOp extends LoopUtil {
    public TFPipeline TFTeleop;

    public boolean firstSave1 = false;
    //Save Position Variables
    public double[] savedX = new double[]{10, 10, 10};
    public double[] savedY = new double[]{10, 10, 10};
    public double[] savedR = new double[]{0.5, 0.5, 0.5};
    public int saveStateIndex = 0;

    public ThreeJointArm newPropArm;

    public static ServoFTC servoA, servoB, servoC, servoD, servoR;
    public static ServoConfig configA, configB, configC, configD, configR;
    public static AngleConversion servoConversionA, servoConversionB, servoConversionC;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;
    public static double commandedPositionMultiplier;
    public Vector2d betterCommandedPosition;

    public static CustomMecanumDrive drive  = null;
    public static Vector3d joystick = new Vector3d();
    public static Vector2d right_stick = new Vector2d();
    public static double[] angles = null;
    public static int angleIndex = 0;
    public static boolean lastStateLB = false, lastStateRB = false,currentStateLB = false, currentStateRB = false;

    public InputHandler inputHandler;
    public SlideController controller;

    public DcMotor left, right;

    public boolean DOpen = false;
    double DPos = DOpen ? 0.01 : 0.9;
    public double R = 0.5;

    public double RunnableTimer = 0;
    public boolean PickUpRunning = false;
    public boolean autoStack = false;
    public double autoStackTimer = 0d;
    public boolean wheelLock = false;
    public double TFsavedTime = 0;
    public boolean TFTime = false;

    //Algorithm-based correction (not PID)
    public static AlgorithmicCorrection correction;

//    private static final double SPEED_REDUCTION = 0.6;

    Runnable setArmToPlace = () -> {
        betterCommandedPosition.x = 26;
        betterCommandedPosition.y = 5;
        R = 0.5;
        DOpen = false;
    };
    Runnable setArmToStow = () -> {
        betterCommandedPosition.x = 2;
        betterCommandedPosition.y = 20;
        R = 0.5;
        DOpen = false;
    };
    Runnable setArmToIntake = () -> {
        betterCommandedPosition.x = 26;
        betterCommandedPosition.y = 5;
        R = 0.5;
        DOpen = true;
    };
    Runnable pickUp = () -> {
        if(RunnableTimer < 0.75* EULConstants.SEC2MS){
            betterCommandedPosition.x = 20;
            DOpen = true;
        }else if(RunnableTimer < 1.25* EULConstants.SEC2MS){
            DOpen = false;
        }else if(RunnableTimer < 1.4* EULConstants.SEC2MS){
            setArmToStow.run();
        }else{
            PickUpRunning = false;
        }
    };
    Runnable stepperLower = () -> {
        if(RunnableTimer > 1* EULConstants.SEC2MS){
            RunnableTimer = 0;
         }
    };
    Runnable loadPoint = () -> {
        betterCommandedPosition.x = savedX[saveStateIndex];
        betterCommandedPosition.y = savedY[saveStateIndex];
        R = savedR[saveStateIndex];
    };
    Runnable savePoint = () -> {
        savedX[saveStateIndex] = betterCommandedPosition.x;
        savedY[saveStateIndex] = betterCommandedPosition.y;
        savedR[saveStateIndex] = R;
    };
    Runnable index = () -> {
        saveStateIndex = Math.abs((saveStateIndex-1) % 3);
    };

//    public Runnable[] commandList = new Runnable[]{loadPoint, pickUp, setArmToStow, highPlace, index, loadPoint, toggleClaw, setArmToStow, groundPlace, index};
    public int autoStackIndex = 0;
    private double angleOffset;
/*
    public void AutoStack() {
        if((autoStackIndex == 0 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 1 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 2 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 3 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 4 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 5 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 6 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 7 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 8 && autoStackTimer > 0*EULConstants.SEC2MS) || (autoStackIndex == 9 && autoStackTimer > 0*EULConstants.SEC2MS)){
            autoStackIndex = (autoStackIndex + 1) % 10;

        }
        commandList[autoStackIndex].run();
    }
*/

    @Override
    public void opInit() {

        //Pre-Defined Arm/Slide Movements and Positions


        //Arm init
        configA = new ServoConfig("A",false, 0.0001, 0.83);
        configB = new ServoConfig("B",true, 0.0001, 0.9999);
        configC = new ServoConfig("C",true, 0.0001, 0.9999);
        configD = new ServoConfig("D",true, 0.0001, 0.9999);
        configR = new ServoConfig("R",true, 0.0001, 0.9999);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);
        servoD = new ServoFTC(hardwareMap, telemetry, configD);
        servoR = new ServoFTC(hardwareMap, telemetry, configR);

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
                new ServoFTC[]{servoA, servoB, servoC},
                new AngleConversion[]{servoConversionA, servoConversionB, servoConversionC},
                15,
                17);
        newPropArm.bindTelemetry(telemetry);

        //Slide init
        inputHandler = InputAutoMapper.normal.autoMap(this);
        //controller = new SlideController(hardwareMap);

        //Drive init
        Globals.opmode(this);
        Globals.input(this);

        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        // Flipping the motor direction is needed since this robot uses NeveRest motors which turn opposite of the GoBilda motors
        drive.mapMotors("FL", !false, "BL", !true, "FR", !false, "BR", !true, false);

        joystick = new Vector3d();
        right_stick = new Vector2d(0, 1);

        lastStateRB = false;
        lastStateLB = false;
        currentStateLB = false;
        currentStateRB = false;

        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        TFTeleop = new TFPipeline(hardwareMap, "Webcam1", new String[]{"Junction Top"});
        TFTeleop.init();

        AngleOffsetHandler offsetHandler = new AngleOffsetHandler();
        try {
            angleOffset = offsetHandler.fromXML();
        } catch (FileNotFoundException e) {
            angleOffset = 0;
        }
    }

    @Override
    public void opInitLoop() {
        if (inputHandler.value("D1:RT") >= 0.5) angleOffset = 0;
    }

    @Override
    public void opStart() {}

    @Override
    public void opStop() {}

    @Override
    public void opUpdate(double deltaTime) {
        if((savedX[0] == 10 && savedX[1] != 10) || (savedX[0] != 10 && savedX[1] == 10)){
            if(savedX[1] != 10){
                firstSave1 = true;
            }
        }

        if(wheelLock){
            joystick.x = 0;
            joystick.y = 0;
            joystick.z = 0;
        }
        handleInput(deltaTime);

        //if (gamepadHandler.up("D2:RT")){
        //    autoStack = !autoStack;
        //    autoStackTimer = 0d;
        //}
        armUpdate(deltaTime);
        outputTelemetry();
        DPos = DOpen ? 0.01 : 0.9;
        if(Double.isNaN(DPos)){DPos=0.6;}
        if(Double.isNaN(R)){R=0.5;}

        servoD.setPosition(DPos);
        servoR.setPosition(R);
        RunnableTimer += deltaTime;
        autoStackTimer += deltaTime;
        telemetry.addData("Delta Time", deltaTime);
        if(TFTime){
            TFsavedTime += deltaTime;
        } else {
            TFsavedTime = 0;
        }



    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        driveFixedUpdate(deltaTime);
    }

    public void armUpdate(double deltaTime) {
        if(betterCommandedPosition.x < 0.001){ betterCommandedPosition.x = 0.001; }
        //newPropArm.propagate(betterCommandedPosition, new Vector2d( 1, 0),true);
        newPropArm.circleFindArmTwo(betterCommandedPosition);
        if(PickUpRunning){
            pickUp.run();
        }
        //if(StepperLowerRunning){
        //    stepperLower.run();
        //}
    }

    public void driveFixedUpdate(double deltaTime){

        lastStateRB = currentStateRB;
        lastStateLB = currentStateLB;

        currentStateLB = gamepad1.left_bumper;
        if(currentStateLB && !lastStateLB){
            angleIndex++;
        }

        currentStateRB = gamepad1.right_bumper;
        if(currentStateRB && !lastStateRB){
            angleIndex--;
        }

        if(angleIndex <= -1){
            angleIndex = 3;
        } else if (angleIndex >= 4){
            angleIndex = 0;
        }
        if(!wheelLock) {
            joystick.x = gamepad1.left_stick_x * -0.6 * (controller.isInUse() ? 0.2 : 1);
            joystick.y = -gamepad1.left_stick_y * -0.6 * (controller.isInUse() ? 0.2 : 1);

            right_stick.x = -gamepad1.right_stick_x;
            right_stick.y = gamepad1.right_stick_y;

            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, right_stick, false);
        }

        telemetry.addData("Joystick X: ", joystick.x);
        telemetry.addData("Joystick Y: ", joystick.y);

        joystick.z = correction.getOutput() * (controller.isInUse() ? 0.2 : 0.5);
        drive.update(joystick, true, deltaTime, angleOffset);
    }

    public void handleInput(double deltaTime){
        inputHandler.loop();
        //D Control
        if (inputHandler.up("D2:RB")){
            DOpen = !DOpen;
        }
        //Slide controls
        if (gamepad2.a){

            if(firstSave1){
                betterCommandedPosition.x = savedX[1];
                betterCommandedPosition.y = savedY[1];
                R = savedR[1];
            }else{
                betterCommandedPosition.x = savedX[0];
                betterCommandedPosition.y = savedY[0];
                R = savedR[0];
            }
        }

        //arm controls
        gamepadHandler.loop();
        if (gamepadHandler.up("D2:LB")){
            RunnableTimer = 0;
            PickUpRunning = true;
        }
        if (gamepadHandler.up("D1:LT")){
            wheelLock = !wheelLock;
        }
        if (gamepadHandler.up("D2:RT")) {
            TFTime = true;
            TFTeleop.scan();
        }
        if (TFsavedTime > 1000) {
            TFTeleop.scoreCone(deltaTime, new Vector2d(0, 0), newPropArm, servoR, servoD);
            TFTime = false;
        }
        if(gamepadHandler.up("D2:DPAD_DOWN") && gamepadHandler.value("D2:RT") > 0.3){
            saveStateIndex = 3;
            savedX[saveStateIndex] = betterCommandedPosition.x;
            savedY[saveStateIndex] = betterCommandedPosition.y;
            savedR[saveStateIndex] = R;
        } else if (gamepadHandler.up("D2:DPAD_DOWN") && gamepadHandler.value("D2:RT") < 0.3){ //decrease outputSpeed by decimalPlace | now wrong comment
            saveStateIndex = 3;
            betterCommandedPosition.x = savedX[saveStateIndex];
            betterCommandedPosition.y = savedY[saveStateIndex];
            R = savedR[saveStateIndex];
        }

        if(gamepadHandler.up("D2:DPAD_UP") && gamepadHandler.value("D2:RT") > 0.3){
            saveStateIndex = 1;
            savedX[saveStateIndex] = betterCommandedPosition.x;
            savedY[saveStateIndex] = betterCommandedPosition.y;
            savedR[saveStateIndex] = R;
        } else if (gamepadHandler.up("D2:DPAD_UP") && gamepadHandler.value("D2:RT") < 0.3){ //increase outputSpeed by decimalPlace | now wrong comment
            saveStateIndex = 1;
            betterCommandedPosition.x = savedX[saveStateIndex];
            betterCommandedPosition.y = savedY[saveStateIndex];
            R = savedR[saveStateIndex];
        }

        if(gamepadHandler.up("D2:DPAD_LEFT") && gamepadHandler.value("D2:RT") > 0.3) {
            saveStateIndex = 0;
            savedX[saveStateIndex] = betterCommandedPosition.x;
            savedY[saveStateIndex] = betterCommandedPosition.y;
            savedR[saveStateIndex] = R;
        } else if (gamepadHandler.up("D2:DPAD_LEFT") && gamepadHandler.value("D2:RT") < 0.3){ //increase outputSpeed by decimalPlace | now wrong comment
            saveStateIndex = 0;
            betterCommandedPosition.x = savedX[saveStateIndex];
            betterCommandedPosition.y = savedY[saveStateIndex];
            R = savedR[saveStateIndex];
        }
        if(gamepadHandler.up("D2:DPAD_RIGHT") && gamepadHandler.value("D2:RT") > 0.3){
            saveStateIndex = 2;
            savedX[saveStateIndex] = betterCommandedPosition.x;
            savedY[saveStateIndex] = betterCommandedPosition.y;
            savedR[saveStateIndex] = R;
        } else if (gamepadHandler.up("D2:DPAD_RIGHT") && gamepadHandler.value("D2:RT") < 0.3){ //decrease outputSpeed by decimalPlace | now wrong comment
            saveStateIndex = 2;
            betterCommandedPosition.x = savedX[saveStateIndex];
            betterCommandedPosition.y = savedY[saveStateIndex];
            R = savedR[saveStateIndex];
        }

        betterCommandedPosition = betterCommandedPosition.plus((new Vector2d(gamepad2.left_stick_y, -gamepad2.right_stick_y*1.5).times(0.5)));
        betterCommandedPosition.x = EULMathEx.doubleClamp(-32, 32, betterCommandedPosition.x);
        betterCommandedPosition.y = EULMathEx.doubleClamp(-20, 32, betterCommandedPosition.y);
        R = EULMathEx.doubleClamp(0.001, 0.999, R+gamepad2.left_stick_x*0.015);
    }


    public void outputTelemetry(){
        telemetry.addData("D control: ", DOpen);
        telemetry.addData("Commanded Multiplier: ", commandedPositionMultiplier);
        telemetry.addData("Commanded Position: ", betterCommandedPosition);
        telemetry.addData("Servo A Turn Position: ", servoA.getPosition());
        telemetry.addData("Servo B Turn Position: ", servoB.getPosition());
        telemetry.addData("Servo C Turn Position: ", servoC.getPosition());
        telemetry.addData("Servo D Turn Position: ", servoD.getPosition());
        telemetry.addData("Current State Index", saveStateIndex);
        telemetry.addData("Is running pick up:", PickUpRunning);
        drive.logMotorPos(telemetry);
    }
}
