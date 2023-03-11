package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSMasterController;

public abstract class EnhancedOpMode extends OpMode {

    private DSMasterController masterController;
    private DSController driver1, driver2;

    private double
            initializedTime,
            deltaTimeMS,
            loopStartTime,
            loopEndTime,
            unprocessedTime,
            frameTime, frames, fps,
            updateCap = 30,
            firstLoopTime;

    public abstract void opInit();
    public abstract void opSetupControllers(DSController driver1, DSController driver2);
    public abstract void opInitLoop();
    public abstract void opStart();
    public abstract void opInput(DSController driver1, DSController driver2, double deltaTimeMS);
    public abstract void opUpdate(double deltaTimeMS);
    public abstract void opFixedUpdate(double deltaTimeMS);
    public abstract void opStop();

    @Override
    public void init() {
        this.initializedTime = System.currentTimeMillis();
        this.firstLoopTime = System.currentTimeMillis();

        this.masterController = new DSMasterController(this);
        this.driver1 = this.masterController.getDriver1();
        this.driver2 = this.masterController.getDriver2();

        this.opInit();
        this.opSetupControllers(driver1, driver2);
    }

    @Override
    public void start(){
        double t = System.currentTimeMillis();
        this.loopStartTime = t;
        this.loopEndTime = t;
        this.firstLoopTime = t;
        opStart();
    }

    @Override
    public void init_loop(){
        opInitLoop();
    }

    @Override
    public void loop() {

        //record the start of a fresh loop
        loopStartTime = System.currentTimeMillis();

        //calculate the difference of this loop's start time from the last loop's start time
        deltaTimeMS = loopStartTime - loopEndTime;

        //set this loop's end time to the start
        loopEndTime = loopStartTime;

        //increment delta for accurate time keeping without constantly calling the loop_timer.milliseconds()
        unprocessedTime += deltaTimeMS;

        //extra for logging additional data
        frameTime += deltaTimeMS;

        opInput(driver1, driver2, deltaTimeMS);
        opUpdate(deltaTimeMS);


        while (unprocessedTime >= updateCap){
            //everytime this loops, it will try to update as many times as what a perfect loop will update every second;
            //this will limit the loop so it doesn't create an infinite loop and stall out the OpMode
            unprocessedTime -= updateCap;

            if (frameTime >= 1.0){
                //extra for logging additional data
                frameTime = 0;
                fps = frames;
                frames = 0;

                //pass in the delta time and current time for the abstract method to use
                opFixedUpdate(updateCap * EULConstants.SEC2MS);

            }
        }
    }

    @Override
    public void stop(){
        opStop();
    }

    public double getDeltaTimeNANO(){
        return deltaTimeMS * EULConstants.MS2NANO;
    }

    public double getDeltaTimeMS(){
        return deltaTimeMS;
    }

    public double getDeltaTimeSEC(){
        return deltaTimeMS * EULConstants.MS2SEC;
    }

    public double getFps(){
        return fps;
    }

    public double getElapsedTimeNANO(){
        return getElapsedTimeMS() * EULConstants.MS2NANO;
    }

    public double getElapsedTimeMS(){
        return System.currentTimeMillis() - firstLoopTime;
    }

    public double getElapsedTimeSEC(){
        return getElapsedTimeMS() * EULConstants.MS2SEC;
    }


    public double getElapsedOpModeTimeNANO(){
        return getElapsedOpModeTimeMS() * EULConstants.MS2NANO;
    }

    public double getElapsedOpModeTimeMS(){
        return System.currentTimeMillis() - initializedTime;
    }

    public double getElapsedOpModeTimeSEC(){
        return getElapsedOpModeTimeMS() * EULConstants.MS2SEC;
    }

    public void setFixedUpdateCap(int cap){
        this.updateCap = 1d / cap;
    }

    public HardwareMap getHardwareMap(){
        return this.hardwareMap;
    }

    public Telemetry getTelemetry(){
        return this.telemetry;
    }
}
