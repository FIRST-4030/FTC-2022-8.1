package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.AngleOffsetHandler;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.PowerPlayGlobals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.AngleConversion;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.servos.kinematics.VirtualServo;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.statemachine.OpState;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.statemachine.OpStateList;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.velocityramping.VelocityRampStepper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement.velocityramping.VelocityRamping;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production.TaskManagerStateMachineDemo;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.general.misc.RunOnce;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.Conditional;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.TaskManager;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

import java.util.Objects;

import javax.xml.bind.JAXBException;

@Autonomous(name = "MecanumAuto")
public class MecanumAuto extends LoopUtil {
    //State Machine
    public TaskManager stateMachine;

    //Controls Declaration for Arm and Slide
    public ThreeJointArm newPropArm;

    public static ServoFTC servoA, servoB, servoC, servoD, servoR;
    public static ServoConfig configA, configB, configC, configD, configR;
    public static AngleConversion servoConversionA, servoConversionB, servoConversionC;
    public DcMotor left, right;
    SlideController.LEVEL slideLevelAuto;

    public static double commandedPositionMultiplier;
    public Vector2d betterCommandedPosition;

    public static SlideController slide;

    public static CustomMecanumDrive drive;
    public static AlgorithmicCorrection correction;
    public static Vector3d motion;
    public static OpStateList stateList;
    double storedDeltaTime;
    double elapsedTime;
    public static RevColorRange RCR2;
    public static ColorView.CMYcolors SeenColor;
    public static ColorView CV2;
    public double savedB, savedG, savedR;
    //
    public static double ColorT1;
    public static boolean checked;
    public VelocityRamping forwardRamp, strafeRamp;
    public VelocityRampStepper stepper;
    double elapsedTimeCycle = 0;
    double elapsedTimeCycleAcum = 0;
    double savedTimeCycle;
    double topConeY = 0;

    public InputHandler inputHandler;
    public static InputHandler gamepadHandler;
    public int startTick = 0, endTick = 0;
    public double startTime = 0, endTime = 0;

    public boolean startRight = true;

    Runnable Idle = () -> {
        motion.x = 0;
        motion.y = 0;
    };

    public RunOnce[] movts = new RunOnce[]{
            new RunOnce() {
                @Override
                public void run() {
                    drive.moveToPos(new Vector3d(0, 0.52, 0));
                    slideLevelAuto = SlideController.LEVEL.REST;
                    betterCommandedPosition.x = 3;
                    betterCommandedPosition.y = 25;
                }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0, 0)); double startTime = elapsedTime;}
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0.81, 0)); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0, (startRight ? 90 : -90))); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, -0.38, 0)); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0, 0)); savedTimeCycle = elapsedTime;}
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0.35, 0)); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0.95, 0)); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, -0.21, 0)); }
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0, 0)); double startTime = elapsedTime;}
            },
            new RunOnce() {
                @Override
                public void run() { drive.moveToPos(new Vector3d(0, 0, 0)); savedTimeCycle = elapsedTime; double startTime = elapsedTime; }
            }
    };

    @Override
    public void opInit() {
        //State Machine
        stateMachine = new TaskManager();
        stateMachine.alwaysRun = () -> {inputHandler.loop();};
        stateMachine.addStates(
                () -> {
                    movts[0].update();
                },
                () -> {
                    movts[1].update();
                    SeenColor = CV2.getColor();
                    savedB = CV2.colorInput.blue*255;
                    savedR = CV2.colorInput.red*255;
                    savedG = CV2.colorInput.green*255;
                },
                () -> {
                    movts[2].update();
                },
                () -> {
                    movts[3].update();
                },
                () -> {
                    movts[4].update();
                },
                () -> {
                    movts[5].update();
                    //cycle(getDeltaTime());
                },
                () -> {
                    movts[6].update();
                },
                () -> {
                    movts[7].update();
                },
                () -> {
                    movts[8].update();
                },
                () -> {
                    movts[9].update();
                },
                () -> {
                    movts[10].update();
                    shortPreLoad(getDeltaTime());
                }
        );
        stateMachine.addConditions(
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{0};
                    }

                    @Override
                    public void check() {
                        if (Objects.requireNonNull(drive.getMotorMap().get("FR")).getCurrentPosition() < 900 && Objects.requireNonNull(drive.getMotorMap().get("FR")).getCurrentPosition() > 750) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{10};
                    }

                    @Override
                    public void check() {
                        if (startTime+5000<elapsedTime) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{1};
                    }

                    @Override
                    public void check() {
                        if (startTime+1200<elapsedTime) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{2};
                    }

                    @Override
                    public void check() {
                        if (Objects.requireNonNull(!Objects.requireNonNull(drive.getMotorMap().get("FR")).isBusy())){
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{9};
                    }

                    @Override
                    public void check() {
                        if (startTime+1000<elapsedTime) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{3};
                    }

                    @Override
                    public void check() {
                        if (!Objects.requireNonNull(drive.getMotorMap().get("FR")).isBusy()) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{9};
                    }

                    @Override
                    public void check() {
                        if (startTime+1500<elapsedTime) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{4};
                    }

                    @Override
                    public void check() {
                        if (!Objects.requireNonNull(drive.getMotorMap().get("FR")).isBusy()) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{5};
                    }

                    @Override
                    public void check() {
                        this.status = STATUS.PASSED;
                        if (elapsedTime > 27000) {
                            status = STATUS.PASSED;
                        } else {
                            //status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{6};
                    }

                    @Override
                    public void check() {
                        if (SeenColor != ColorView.CMYcolors.GREEN) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{8};
                    }

                    @Override
                    public void check() {
                        if (SeenColor != ColorView.CMYcolors.BLUE && startRight || SeenColor != ColorView.CMYcolors.RED && !startRight) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                },
                new Conditional() {
                    @Override
                    public void init() {
                        linkedStates = new int[]{7};
                    }

                    @Override
                    public void check() {
                        if (SeenColor != ColorView.CMYcolors.RED && startRight || SeenColor != ColorView.CMYcolors.BLUE && !startRight) {
                            status = STATUS.PASSED;
                        } else {
                            status = STATUS.FAILED;
                        }
                    }
                }
        );
        //
        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        //Slide
        slide = new SlideController(hardwareMap, "LSRM", false);
        //Velocity Ramps
        forwardRamp = new VelocityRamping(PowerPlayGlobals.MAX_VELOCITY);
        strafeRamp = new VelocityRamping(PowerPlayGlobals.MAX_VELOCITY);
        //Misc
        elapsedTime = 0;
        storedDeltaTime = 0;
        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());
        checked = false;
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //Slide and Arm Auto Init
        configA = new ServoConfig("A",false, 0, 0.83);
        configB = new ServoConfig("B",true, 0, 1);
        configC = new ServoConfig("C",true, 0, 1);
        configD = new ServoConfig("D",true, 0, 1);
        configR = new ServoConfig("R",true, 0, 1);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);
        servoD = new ServoFTC(hardwareMap, telemetry, configD);
        servoR = new ServoFTC(hardwareMap, telemetry, configR);

        servoConversionA = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionB = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionC = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);

        betterCommandedPosition = new Vector2d(10,15);
        commandedPositionMultiplier = 1;

        newPropArm = new ThreeJointArm(
                new VirtualServo(15, new Vector2d(0, -1), new Vector2d(0, 0)),
                new ServoFTC[]{servoA, servoB, servoC},
                new AngleConversion[]{servoConversionA, servoConversionB, servoConversionC},
                15,
                17);
        newPropArm.bindTelemetry(telemetry);

        elapsedTimeCycle = 0;
        elapsedTimeCycleAcum = 0;
        topConeY = -0.2;

        //Slide init
        inputHandler = InputAutoMapper.normal.autoMap(this);
        slide = new SlideController(hardwareMap, "LSRM", true);
        right = slide.getRight();

        slideLevelAuto = SlideController.LEVEL.REST;

        //Drive controls movement
        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true, true);

        //Correction outputs calculated turn speed
        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        //Motion controls current wheel movement
        motion = new Vector3d();

        stepper = new VelocityRampStepper(forwardRamp, strafeRamp);
        stepper.addRampForward(1, 1.25, 1.75);
        stepper.addRampForward(1, 1.25, 1.75);


        Runnable driveUpdate = () -> {

        };


        //STATELIST IS NOW OUTDATED ||| StateList is the list of all states the robot will occupy in autonomous
        stateList = new OpStateList();

        stateList.addStates(
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, Math.PI, true);
                            motion.z = correction.getOutput();
                            motion.x = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        Idle,
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2 * (startRight ? 1 : -1), true);
                            motion.z = correction.getOutput();
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2 * (startRight ? 1 : -1), true);
                            motion.z = correction.getOutput();
                            motion.x = -1  * (startRight ? 1 : -1);
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2  * (startRight ? 1 : -1), true);
                            motion.z = correction.getOutput();
                            motion.x = 0.8  * (startRight ? 1 : -1);
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        Idle,
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, Math.PI * (startRight ? 1 : -1), true);
                            motion.z = correction.getOutput();
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            drive.update(motion, true, storedDeltaTime);
                        }
                )
        );

        AngleOffsetHandler offsetHandler = new AngleOffsetHandler();
        offsetHandler.recordAngle(drive.getImu());
        try {
            offsetHandler.toXML();
        } catch (JAXBException e){
            telemetry.addData("FAILED TO WRITE TO XML: ", e.toString());
        }
    }

    public void shortPreLoad(double deltaTime){
        if (elapsedTimeCycle < 0.5 * EULConstants.SEC2MS) {
            slideLevelAuto = SlideController.LEVEL.REST;
            betterCommandedPosition.x = 2;
            betterCommandedPosition.y = 25;
        } else if (elapsedTimeCycle < 1.9 * EULConstants.SEC2MS) {
            servoR.setPosition((startRight ? 1 : 0));
            betterCommandedPosition.x = 18;
            betterCommandedPosition.y = 21;
        } else if (elapsedTimeCycle < 2.2 * EULConstants.SEC2MS) {
            servoD.setPosition(0.07);
        } else if (elapsedTimeCycle < 2.7 * EULConstants.SEC2MS) {
            betterCommandedPosition.x = 1;
            betterCommandedPosition.y = 20;
            servoR.setPosition(0.05);
            servoD.setPosition(0.7);
        }
        elapsedTimeCycle += deltaTime;
    }

    public void cycle(double deltaTime) { //Cycle 4 cones, 24.5 seconds

        if (elapsedTimeCycle < 1 * EULConstants.SEC2MS) {
            slideLevelAuto = SlideController.LEVEL.HIGH;
            betterCommandedPosition.x = 2;
            betterCommandedPosition.y = 25;
            servoR.setPosition(0.3);
            servoD.setPosition(0.6);
        } else if (elapsedTimeCycle < 2.1 * EULConstants.SEC2MS) {
            servoR.setPosition((startRight ? 0.15 : 0.85));
            betterCommandedPosition.x = 23;
            betterCommandedPosition.y = 15;
        } else if (elapsedTimeCycle < 2.4 * EULConstants.SEC2MS) {
            servoD.setPosition(0.07);
        } else if (elapsedTimeCycle < 5 * EULConstants.SEC2MS) {

            betterCommandedPosition.x = 2;
            betterCommandedPosition.y = 20;
            servoR.setPosition(0.3);
        }
        /*
        if(elapsedTimeCycleAcum < (((27-(savedTimeCycle * EULConstants.MS2SEC)) - (4 + 1)) * EULConstants.SEC2MS)) { //(Total Time - (Cycle Time + Buffer))
            if (elapsedTimeCycle < 1 * EULConstants.SEC2MS) {
                slideLevelAuto = SlideController.LEVEL.HIGH;
                betterCommandedPosition.x = 2;
                betterCommandedPosition.y = 25;
                servoR.setPosition(0.3);
                servoD.setPosition(0.6);
            } else if (elapsedTimeCycle < 1.6 * EULConstants.SEC2MS) {
                servoR.setPosition((startRight ? 0 : 1));
                betterCommandedPosition.x = 17;
                betterCommandedPosition.y = 20;
            } else if (elapsedTimeCycle < 1.8 * EULConstants.SEC2MS) {
                servoD.setPosition(0.07);
            } else if (elapsedTimeCycle < 2.2 * EULConstants.SEC2MS) {
                betterCommandedPosition.x = 2;
                betterCommandedPosition.y = 25;
                servoR.setPosition(0.5);
            } else if (elapsedTimeCycle < 3.2 * EULConstants.SEC2MS) {
                slideLevelAuto = SlideController.LEVEL.REST;
                betterCommandedPosition.x = 3;
                betterCommandedPosition.y = 25;
            } else if (elapsedTimeCycle < 3.4 * EULConstants.SEC2MS) {
                betterCommandedPosition.y = topConeY;
            } else if (elapsedTimeCycle < 3.8 * EULConstants.SEC2MS) {
                servoD.setPosition(0.6);
            } else if (elapsedTimeCycle < 4 * EULConstants.SEC2MS) {
                betterCommandedPosition.y = 20;
            } else {
                topConeY -= 3.4;
                elapsedTimeCycle = 0;
            }
        } else {
            servoD.setPosition(0.6);
            slideLevelAuto = SlideController.LEVEL.REST;
            servoR.setPosition(0.5);
            betterCommandedPosition.x = 15;
            betterCommandedPosition.y = 15;
        }
         */
        elapsedTimeCycle += deltaTime;
        elapsedTimeCycleAcum += deltaTime;
    }

    @Override
    public void opInitLoop() {
        gamepadHandler.loop();
        if (gamepadHandler.up("D2:DPAD_LEFT")){ //increase outputSpeed by decimalPlace | now wrong comment
            startRight = false;
        }
        if (gamepadHandler.up("D2:DPAD_RIGHT")){ //decrease outputSpeed by decimalPlace | now wrong comment
            startRight = true;
        }

        telemetry.addData("Starting Side: ", (startRight ? "Right" : "Left"));
        telemetry.addData("DPad L/R to change side", "");
        servoD.setPosition(0.7);
    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        if (elapsedTime > 0.5*EULConstants.SEC2MS){
            newPropArm.circleFind(betterCommandedPosition);
        }
        slide.update(deltaTime, slideLevelAuto, 1);
        stateMachine.execute();
        drive.posUpdate(0.25);
        //STATELOOP IS OUTDATED
        /*
        if (elapsedTime < 1.75 * EULConstants.SEC2MS) { //Drive Forward
            betterCommandedPosition.x = 5;
            betterCommandedPosition.y = 20;
            motion.y = EULMathEx.doubleClamp(-0.999, 0.999, motion.y);
            motion.y = -stepper.update(deltaTime * EULConstants.MS2SEC)[0];
            stateList.setIndex(0);
            servoD.setPosition(0.6);
        }else if (elapsedTime < 2.5*EULConstants.SEC2MS) { //Idle
            stateList.setIndex(0);
            motion.y = 0;
        }else if (elapsedTime < 5*EULConstants.SEC2MS && SeenColor == ColorView.CMYcolors.YELLOW) { //Cycle
            stateList.setIndex(5);
            motion.x = 0.2 * (startRight ? 1 : -1);
        }else if (elapsedTime < 5*EULConstants.SEC2MS && SeenColor == ColorView.CMYcolors.CYAN) { //Cycle
            motion.x = -0.2 * (startRight ? 1 : -1);
        }else { // Stay in Cyan
            motion.y = 0;
            motion.x = 0;
        }
         */

        inputHandler.loop();
        storedDeltaTime = deltaTime;
        elapsedTime += deltaTime;
        CV2.update(RCR2.color(), RCR2.distance());
        /*
        if (elapsedTime > 2*EULConstants.SEC2MS){
            newPropArm.circleFind(betterCommandedPosition);
        }
        slide.update(deltaTime, slideLevelAuto, 1);
        //All comments in StateLoop are wrong, correct later
        if (elapsedTime < 1.75 * EULConstants.SEC2MS) {//Drive Forward
            betterCommandedPosition.x = 5;
            betterCommandedPosition.y = 20;
            motion.y = -stepper.update(deltaTime * EULConstants.MS2SEC)[0];
            stateList.setIndex(0);
            servoD.setPosition(0.6);
        }else if (elapsedTime < 2.25*EULConstants.SEC2MS){ //Halted Turn
            stateList.setIndex(1);
        }else if (elapsedTime < 2.65*EULConstants.SEC2MS){ //
            stateList.setIndex(2);
        }else if (elapsedTime < 3.35*EULConstants.SEC2MS) { //Idle
            stateList.setIndex(1);
        }else if (elapsedTime < 27.85*EULConstants.SEC2MS) { //Cycle
            //Idle.run();
            //cycle(deltaTime);
        }else if (elapsedTime < 28.95*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.YELLOW){ // Move to Yellow
            stateList.setIndex(3);
            betterCommandedPosition.y = 25;
            betterCommandedPosition.x = 5;
        }else if (elapsedTime < 28.5*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.MAGENTA) { // Move to Magenta
            stateList.setIndex(3);
            betterCommandedPosition.y = 25;
            betterCommandedPosition.x = 5;
        }else { // Stay in Cyan
            betterCommandedPosition.y = 25;
            betterCommandedPosition.x = 5;
            stateList.setIndex(4);
            motion.x = -0.5;
        }

         */

        if (inputHandler.up("D1:LB")){
            startTick = drive.getMotor(0).getCurrentPosition();
            startTime = elapsedTime;
        }

        if (inputHandler.up("D1:RB")){
            endTick = drive.getMotor(0).getCurrentPosition();
            endTime = elapsedTime;
        }

        //stateList.getCurrentState().runAll(deltaTime);
        telemetry.addData("Time: ", elapsedTime * EULConstants.MS2SEC);
        telemetry.addData("State Index: ", stateList.getIndex());
        telemetry.addData("Saved Color: ", SeenColor);
        telemetry.addData("Dist: ", RCR2.distance());
        telemetry.addData("ColorBetter: ", CV2.getColorBetter(80));
        telemetry.addData("Color: ", CV2.getColor());
        telemetry.addData("Motion.y", motion.y);

        telemetry.addData("VelocityRecording: ", "");
        telemetry.addData("Start Tick: ", startTick);
        telemetry.addData("End Tick: ", endTick);
        telemetry.addData("Absolute Delta: ", Math.abs(endTick - startTick));
        telemetry.addData("Start Time: ", startTime);
        telemetry.addData("End Time: ", endTime);
        telemetry.addData("Absolute Delta: ", Math.abs(endTime - startTime));
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("Color Raw", RCR2.color);
        telemetry.addData("Red", CV2.colorInput.red*255);
        telemetry.addData("Green", CV2.colorInput.green*255);
        telemetry.addData("Blue", CV2.colorInput.blue*255);
        telemetry.addData("Saved Red", savedR);
        telemetry.addData("Saved Green", savedG);
        telemetry.addData("Saved Blue", savedB);
        drive.logMotorPos(telemetry);
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
