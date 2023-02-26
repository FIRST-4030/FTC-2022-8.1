package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class RepeatableConditional extends Conditional{

    protected int loopCount = 0;
    protected int loopLimit;

    public ArrayList<Conditional> conditionals;
    public boolean operatedConditionEnded = true;
    public Conditional operatedConditional = null;
    public boolean operatedConditionalInitialized = false;

    private boolean validity = true;

    public RepeatableConditional(int loopLimit, Conditional... conditionals){
        this.loopLimit = loopLimit;
        this.conditionals = new ArrayList<>();
        if (conditionals != null){
            this.conditionals.addAll(Arrays.asList(conditionals));
        }
    }

    public void setupStates(int[] linkedStates){
        loopCount = 0;
        operatedConditional = conditionals.get(0);
        loopInit();
    }

    public abstract void loopInit();

    @Override
    public void execute() {

        if (operatedConditionEnded){
            loopCount++;

            if (loopCount >= loopLimit){
                validity = false;
            } else {
                operatedConditional = conditionals.get(loopCount);
            }
        }
        if (validity) {
            if (!operatedConditionalInitialized) {
                operatedConditional.init();
                operatedConditional.setupStates(operatedConditional.linkedStates);
                operatedConditionalInitialized = true;
            }

            operatedConditional.execute();

            if (operatedConditional.isFinished()) {
                operatedConditional.end();
                operatedConditionEnded = true;
            }
        }
    }

    @Override
    public void end(){
        loopCount = 0;
    }

    @Override
    public boolean isFinished() {
        return loopCount < loopLimit;
    }
}
