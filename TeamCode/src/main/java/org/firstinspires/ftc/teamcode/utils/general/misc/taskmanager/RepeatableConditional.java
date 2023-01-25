package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager;

import java.util.ArrayList;

public abstract class RepeatableConditional extends Conditional{

    protected int loopCount = 0;
    protected int loopLimit;

    protected int currentCondition = 0;

    public ArrayList<Conditional> conditionals;
    public STATUS currentConditionStatus = STATUS.FAILED;

    public RepeatableConditional(int loopLimit, Conditional... conditionals){
        this.loopLimit = loopLimit;
        this.conditionals = new ArrayList<>();
        if (conditionals != null){
            for (int i = 0; i < conditionals.length; i++){
                this.conditionals.add(conditionals[i]);
            }
        }
    }

    public void init(){
        loopCount = 0;
        for (Conditional c: conditionals) {
            c.init();
        }
        uInit();
    }

    public abstract void uInit();

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
