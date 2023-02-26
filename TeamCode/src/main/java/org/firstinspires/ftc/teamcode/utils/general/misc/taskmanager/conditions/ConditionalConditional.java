package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions;

import android.os.Build;

import java.util.function.Supplier;

public class ConditionalConditional extends Conditional{

    private Conditional tCondition, fCondition;
    private Supplier<Boolean> conditionFunction;

    public ConditionalConditional(Conditional trueCondition, Conditional falseCondition, Supplier<Boolean> boolProvider){
        this.tCondition = trueCondition;
        this.fCondition = falseCondition;
        this.conditionFunction = boolProvider;
    }

    @Override
    public void init() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (conditionFunction.get()){
                tCondition.init();
            } else {
                fCondition.init();
            }
        }
    }

    @Override
    public void setupStates(int[] linkedStates) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (conditionFunction.get()){
                tCondition.setupStates(linkedStates);
            } else {
                fCondition.setupStates(linkedStates);
            }
        }
    }

    @Override
    public void execute() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (conditionFunction.get()){
                tCondition.execute();
            } else {
                fCondition.execute();
            }
        }
    }

    @Override
    public void end() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (conditionFunction.get()){
                tCondition.end();
            } else {
                fCondition.end();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (conditionFunction.get()){
                return tCondition.isFinished();
            } else {
                return fCondition.isFinished();
            }
        }
        return true;
    }
}
