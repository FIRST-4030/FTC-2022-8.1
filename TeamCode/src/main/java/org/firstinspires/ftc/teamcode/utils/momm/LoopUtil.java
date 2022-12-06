package org.firstinspires.ftc.teamcode.utils.momm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;

import java.util.Vector;

/*
Terminology:
    Delta Time - how long it takes to do something
    Update - Method that executes every single time it's called in a loop/main structure
    Fixed Update - updates that will execute a fixed amount of times per time unit
 */
public abstract class LoopUtil extends OpMode{

    //MOMM integration
    private final Vector<OpMode> opmodes;
    public final InputHandler input;


    protected ElapsedTime loop_timer;

    private boolean
            timeStart = false,
            running = false;

    private double
            delta_time,
            loop_start_time,
            loop_end_time,
            update_wait_time,
            fixed_update_wait_time,
            unprocessed_time,
            frameTime, frames, fps,
            MIN_WAIT_LIMIT, UPDATE_CAP;


    public LoopUtil(){
        Globals.opmode = this;
        opmodes = new Vector<>();
        input = new InputHandler(this);
    }

    /**
     * This method will register an OpMode passed in as the parameter
     * <br>Throws a NullPointerException if you don't pass an OpMode
     * <br>Throws an IllegalArgumentException if you pass LoopUtil into OpMode
     * <br>Throws an IllegalArgumentException if it already has the OpMode registered
     * @param opMode
     */
    public void register(OpMode opMode){
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        if (opMode.equals(this)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "Refusing to re-register the primary OpMode: " + opMode.getClass().getSimpleName());
        }
        if (opmodes.contains(opMode)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "OpMode already registered: " + opMode.getClass().getSimpleName());
        }

        opMode.hardwareMap = hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = gamepad1;
        opMode.gamepad2 = gamepad2;

        opmodes.add(opMode);
    }

    /**
     * This method will deregister the specified OpMode
     * <br>Throws a NullPointerException if there's no OpMode passed as an argument
     * @param opMode
     */
    public void deregister(OpMode opMode) {
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        opmodes.remove(opMode);
    }

    /**
     * init() all registered OpModes
     */
    public void initOpModes(){
        for (OpMode op : opmodes){
            op.time = time;
            op.init();
        }
    }

    /**
     * init_loop() all registered OpModes
     */
    public void initLoopOpModes(){
        input.loop();
        for (OpMode op : opmodes){
            op.time = time;
            op.init_loop();
        }
    }

    /**
     * start() all registered OpModes
     */
    public void startOpModes(){
        for (OpMode op : opmodes){
            op.time = time;
            op.start();
        }
    }

    /**
     * loop() all registered OpModes
     */
    public void loopOpModes(){
        input.loop();
        for (OpMode op : opmodes){
            op.time = time;
            op.loop();
        }
    }

    /**
     * stop() all registered OpModes
     */
    public void stopOpModes(){
        for (OpMode op : opmodes){
            op.time = time;
            op.stop();
        }
    }

    @Override
    public void init() {
        //we make a master loop timer
        loop_timer = new ElapsedTime();

        //Zero out all variables used for looping
        delta_time = 0;
        loop_start_time = 0;
        loop_end_time = 0;

        //Zero out the wait time since
        update_wait_time = 0;
        fixed_update_wait_time = 0;

        frameTime = 0;
        frames = 0;
        fps = 0;

        //set minimum wait time
        MIN_WAIT_LIMIT = 0.1;

        //set fixed updates per second; in this case, the denominator states that there will be 60 updates per second
        UPDATE_CAP = 1.0 / 30.0;

        //let the subclass initialize
        opInit();
    }

    @Override
    public void init_loop(){opInitLoop();}

    @Override
    public void start(){opStart();}

    @Override
    public void loop() {
        //replicate what the start of a game loop is like without using the built-in OpMode func: init()
        if (!timeStart){
            first_loop();
        }

        //record the start of a fresh loop
        loop_start_time = loop_timer.milliseconds();

        //calculate the difference of this loop's start time from the last loop's start time
        delta_time = loop_start_time - loop_end_time;

        //set this loop's end time to the start
        loop_end_time = loop_start_time;

        //increment delta for accurate time keeping without constantly calling the loop_timer.milliseconds()
        unprocessed_time += delta_time;

        //extra for logging additional data
        frameTime += delta_time;


        while (unprocessed_time >= UPDATE_CAP){
            //everytime this loops, it will try to update as many times as what a perfect loop will update every second;
            //this will limit the loop so it doesn't create an infinite loop and stall out the OpMode
            unprocessed_time -= UPDATE_CAP;

            if (frameTime >= 1.0){
                //extra for logging additional data
                frameTime = 0;
                fps = frames;
                frames = 0;

                //checks if the system has skipped the commanded cycles
                if (fixed_update_wait_time < 1) {
                    //pass in the delta time and current time for the abstract method to use
                    opFixedUpdate(UPDATE_CAP * EULConstants.SEC2MS);
                } else {
                    //decrement with every skip so it doesn't not execute the fixed_update when the commanded
                    //frame skips count is up
                    update_wait_time -= 1;
                    if (update_wait_time < 0){ update_wait_time = 0;} //make sure it doesn't go below
                }
            }
        }


        //this will try to update every loop call, which can make it "consistent"
        //so, to realistically not make the OpMode thread sleep, we make it skip the update method instead
        if (update_wait_time < MIN_WAIT_LIMIT) {
            //pass in the delta time and current time for the abstract method to use
            opUpdate(delta_time);
        } else {
            //decrement so that it keeps track of time without multiple calls to the loop_timer.milliseconds() method
            update_wait_time -= delta_time;
        }
    }

    @Override
    public void stop(){opStop();}

    private void first_loop(){
        loop_timer.reset();
        timeStart = true;
        running = true;
    }

    //declare abstract substitutes for OpMode since you can't extend multiple classes
    public abstract void opInit();
    public abstract void opInitLoop();
    public abstract void opStart();
    public abstract void opUpdate(double deltaTime);
    public abstract void opFixedUpdate(double deltaTime);
    public abstract void opStop();

    public void update_wait_for(double ms){
        update_wait_time = ms;
    }

    public void fixed_update_wait_for(double frame_count){
        fixed_update_wait_time = frame_count;
    }

    public double getDeltaTime(){
        return delta_time;
    }

    public double getCurrentTimeMs(){
        return loop_timer.milliseconds();
    }

    public double getFps(){
        return this.fps;
    }

    public double getCurrentTimeSec(){
        return loop_timer.milliseconds() / 1000;
    }

    public boolean isRunning(){
        return running;
    }

    public void setUpdateCap(int updates){
        UPDATE_CAP = 1.0 / updates;
    }
}
