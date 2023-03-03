package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.input;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.opmode.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions.Conditional;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions.ParallelConditionalGroup;
import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

import java.util.Arrays;
import java.util.Vector;

public class DSMasterController implements Runnable{

    public enum Driver{
        DRIVER1,
        DRIVER2
    }

    private EnhancedOpMode opMode;
    private DSController driver1, driver2;

    private final ConditionalManager conditionalManager;

    public DSMasterController(EnhancedOpMode enhancedOpMode){
        this.opMode = enhancedOpMode;

        this.driver1 = new DSController(this.opMode.gamepad1);
        this.driver2 = new DSController(this.opMode.gamepad2);

        this.conditionalManager = new ConditionalManager();
    }

    public DSController getController(Driver driver){
        if (driver == Driver.DRIVER2) {
            return driver2;
        }
        return driver1;
    }

    public ConditionalManager getConditionalManager(){
        return conditionalManager;
    }

    @Override
    public void run() {
        driver1.run();
        driver2.run();

        conditionalManager.addConditionals(findConditionals(driver1), findConditionals(driver2));
        conditionalManager.execute();
    }

    private ParallelConditionalGroup findConditionals(DSController driver){
        Vector<Conditional> driverConditionals = new Vector<>();
        for (DSController.Analog analog: driver.getAnalogList()) {
            if (analog.rest) driverConditionals.add(analog.getRestConditional());
            if (analog.maxed) driverConditionals.add(analog.getMaxedConditional());
            if (analog.changed) driverConditionals.add(analog.getChangedConditional());
        }

        for (DSController.Digital digital: driver.getDigitalList()) {
            if (digital.changed) driverConditionals.add(digital.getChangedConditional());
            if (digital.held) driverConditionals.add(digital.getHeldConditional());
            if (digital.released) driverConditionals.add(digital.getReleasedConditional());
            if (digital.pressed) driverConditionals.add(digital.getPressedConditional());
        }

        return new ParallelConditionalGroup(driverConditionals.toArray(new Conditional[0]));
    }
}
