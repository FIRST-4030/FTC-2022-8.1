package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSController;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input.DSMasterController;

public abstract class EnhancedOpMode extends OpMode {

    private DSMasterController masterController;

    private double
            initializedTime,
            deltaTimeMS,
            loopStartTime,
            loopEndTime,
            unprocessedTime,
            frameTime, frames, fps,
            updateCap = 30;

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
        this.masterController = new DSMasterController(this);

        this.opInit();
        this.opSetupControllers(masterController.getDriver1(), masterController.getDriver2());
    }

    @Override
    public void start(){
        opStart();
    }

    @Override
    public void init_loop(){
        opInitLoop();
    }

    @Override
    public void loop() {
        this.loopStartTime = System.currentTimeMillis();

        //calculate the difference of this loop's start time from the last loop's start time
        this.deltaTimeMS = this.loopStartTime - this.loopEndTime;

        //set this loop's end time to the start
        this.loopEndTime = this.loopStartTime;

        //increment delta for accurate time keeping without constantly calling the loop_timer.milliseconds()
        this.unprocessedTime += this.deltaTimeMS;

        //extra for logging additional data
        this.frameTime += this.deltaTimeMS;

        this.masterController.run();
        opInput(this.masterController.getDriver1(), this.masterController.getDriver2(), this.deltaTimeMS);

        while (this.unprocessedTime >= this.updateCap){
            //everytime this loops, it will try to update as many times as what a perfect loop will update every second;
            //this will limit the loop so it doesn't create an infinite loop and stall out the OpMode
            this.unprocessedTime -= this.updateCap;

            if (this.frameTime >= 1.0){
                //extra for logging additional data
                this.frameTime = 0;
                this.fps = frames;
                this.frames = 0;

                opFixedUpdate(this.deltaTimeMS);
            }
        }
        opUpdate(this.deltaTimeMS);
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
        return System.currentTimeMillis() - initializedTime;
    }

    public double getElapsedTimeSEC(){
        return getElapsedTimeMS() * EULConstants.MS2SEC;
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
