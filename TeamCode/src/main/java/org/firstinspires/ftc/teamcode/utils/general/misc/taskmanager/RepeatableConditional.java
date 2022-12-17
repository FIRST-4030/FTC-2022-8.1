package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Collections;

public abstract class RepeatableConditional extends Conditional{

    protected int loopCount = 0;
    protected int loopLimit = 0;

    protected int currentCondition = 0;

    public ArrayList<Conditional> conditionals;
    public STATUS currentConditionStatus = STATUS.FAILED;

    public RepeatableConditional(int loopLimit, Conditional... conditionals){
        this.loopLimit = loopLimit;
        Collections.addAll(this.conditionals, conditionals);
    }

    public abstract void init();

    @Override
    public void check() {
        if (currentConditionStatus == STATUS.PASSED){
            currentCondition++;
        }

        if (currentCondition >= conditionals.size()){
            loopCount++;
            currentCondition = 0;
        }

        conditionals.get(currentCondition).check();
        currentConditionStatus = conditionals.get(currentCondition).status;
        this.linkedStates = conditionals.get(currentCondition).linkedStates;

        if ((loopLimit - 1) <= loopCount){
            this.status = STATUS.PASSED;
        } else {
            this.status = STATUS.FAILED;
        }
    }
}
