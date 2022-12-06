package org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.TFTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.utils.cvision.tensorflow.depreciated.tfodohm.ODMain.TFODModule;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4fBuilder;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.MathEx;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector2F;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;
import org.firstinspires.ftc.teamcode.utils.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.NewNewDrive;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.general.misc.OrderedEnumHelper;

@Config
@Disabled
@Autonomous(name = "TFDrive", group = "Test")
public class TFDriveTest extends MultiOpModeManager {
    private NewNewDrive drive;
    private TFODModule tfodModule;

    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;

    private final DepreciatedVector3F camera_pos = new DepreciatedVector3F(5.5f, 15.5f, -7.5f);
    private final DepreciatedMatrix4f camera_rot = DepreciatedMatrix4fBuilder.buildGenRot(-53, -185, 2);

    @Override
    public void init() {

        try {
            super.register(new NewNewDrive());
            drive = new NewNewDrive();
            super.register(drive);
            drive.init();
        } catch (Exception e){
            telemetry.log().add(drive.getClass().getSimpleName() + " is not initializing.");
        }

        try{
            super.register(new TFODModule(camera_pos, camera_rot));
            tfodModule = new TFODModule(camera_pos, camera_rot);
            super.register(tfodModule);
            tfodModule.init();
        } catch (Exception e){
            telemetry.log().add(tfodModule.getClass().getSimpleName() + " is not initializing.");
        }

        telemetry.addData("TFObjectDetection Null? ", tfodModule.getTfod() == null ? "Yes" : "No");
        telemetry.addData("Vuforia Null? ", tfodModule.getVuforia() == null ? "Yes" : "No");
        telemetry.addData("IMU Null? ", drive.getImu() == null ? "Yes" : "No");

        telemetry.log().add(tfodModule.calcCoordinate(new DepreciatedVector2F(0,0)).toString());
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        drive.setDoneFalse();
        state = AUTO_STATE.VERIFICATION;
        super.start();
    }

    //Variables needed for looping
    private boolean scanned = false, sorted = false, calculated = false, startedMove = false, reversingMove = false;
    private DepreciatedVector2F tempV2, target;
    private DepreciatedVector3F targetPreCasted;
    private double storedRadius, storedArcLength;
    private boolean inIMG = false;

    public static double speedMin = 0.1;
    public static double speedMax = 0.5;
    @Override
    public void loop() {
        if (oldState != state) {
            drive.setDoneFalse();
            oldState = state;
        }
        switch (state){
            case VERIFICATION:
                inIMG = tfodModule.verifyImg();
                if (inIMG == true){
                    state = AUTO_STATE.SCAN;
                }
                break;

            case SCAN: //scan for objects
                if (!tfodModule.isBusy() && !(scanned)) {
                    tfodModule.scan();
                    scanned = true;
                }
                if (scanned) {state = AUTO_STATE.SORT;}
                break;

            case SORT:
                if (!tfodModule.isBusy() && scanned && !(sorted)){
                    tempV2 = tfodModule.sortCBBB();
                    sorted = true;
                }
                if (sorted) {state = AUTO_STATE.CALC;}
                break;

            case CALC:
                if (!tfodModule.isBusy() && sorted && !(calculated)){
                    targetPreCasted = tfodModule.calcCoordinate(tempV2);
                    calculated = true;
                }
                if (calculated) {state = AUTO_STATE.RESET_VAR;}
                break;

            case RESET_VAR: //reset all variables used to check stuff in the previous case
                scanned = false;
                sorted = false;
                calculated = false;
                startedMove = false;
                reversingMove = false;
                target = new DepreciatedVector2F(targetPreCasted.getX(), targetPreCasted.getZ());
                double[] f = MathEx.makeArcV1(target);
                storedRadius = f[0];
                storedArcLength = f[1];
                inIMG = false;
                state = AUTO_STATE.START_MOVE;
                break;

            case START_MOVE:
                if (startedMove == false){
                    startedMove = true;
                    drive.arcTo(storedRadius, storedArcLength, speedMin, speedMax);
                }
                if (startedMove == true && (!drive.isBusy() && drive.isDone())){
                    startedMove = false;
                    state = AUTO_STATE.REVERSE_MOVE;
                }
                break;

            case REVERSE_MOVE:
                if (reversingMove == false){
                    reversingMove = true;
                    drive.arcTo(-storedRadius, -storedArcLength, speedMin, speedMax);
                }
                if (reversingMove && (!drive.isBusy() && drive.isDone())){
                    reversingMove = false;
                    state = AUTO_STATE.VERIFICATION;
                }
                break;

            case DONE:
                break;
        }
        telemetry.addData("Current State: ", state);
        telemetry.addData("Calculated Vector: ", target);
        telemetry.addData("Scanned: ", scanned);
        telemetry.addData("Sorted: ", sorted);
        telemetry.addData("Calculated: ", calculated);
        telemetry.addData("StartedMove: ", startedMove);
        telemetry.addData("ReversingMove: ", reversingMove);
        telemetry.addData("TargetPreCasted: ", targetPreCasted);
        telemetry.addData("Radius: ", storedRadius);
        telemetry.addData("ArcLength: ", storedArcLength);
    }

    @Override
    public void stop() {
        tfodModule.stop();
        drive.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        VERIFICATION,
        SCAN,
        SORT,
        CALC,
        RESET_VAR,
        START_MOVE,
        REVERSE_MOVE,
        DONE;
        public TFDriveTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}
