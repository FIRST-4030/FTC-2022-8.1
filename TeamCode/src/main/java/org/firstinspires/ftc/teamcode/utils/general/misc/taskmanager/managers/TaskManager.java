package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.managers;

import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions.Conditional;

import java.util.Arrays;
import java.util.Vector;

public class TaskManager {

    public int currentConditional;

    public Vector<Conditional> endConditions;
    public Runnable alwaysRun;
    public Vector<Runnable> states;

    private boolean initializedConditional = false;

    public TaskManager(){
        endConditions = new Vector<>();
        states = new Vector<>();

        alwaysRun = () -> {};

        currentConditional = 0;
    }

    public void addConditions(Conditional... conditionals){
        endConditions.addAll(Arrays.asList(conditionals));
    }

    public void addCondition(Conditional conditional, int idx){
        endConditions.add(idx, conditional);
    }

    public void removeConditions(int... idx){
        int[] sorted = Arrays.copyOf(idx, idx.length);
        Arrays.sort(sorted); //sort in ascending order to later remove back to front

        for (int i = 0; i < sorted.length; i++) {
            endConditions.remove(sorted[sorted.length - 1 - i]); //remove back to front
        }
    }

    public void addStates(Runnable... states){
        this.states.addAll(Arrays.asList(states)); //add all the necessary conditionals
    }

    public void addState(Runnable state, int idx){
        states.add(idx, state);
    }

    public void removeStates(int... idx){
        int[] sorted = Arrays.copyOf(idx, idx.length);
        Arrays.sort(sorted); //sort in ascending order to later remove back to front

        for (int i = 0; i < sorted.length; i++) {
            states.remove(sorted[sorted.length - 1 - i]); //remove back to front
        }
    }

    public void clearConditions(){
        endConditions.clear();
    }

    public void clearStates(){
        states.clear();
    }

    public void execute(){
        alwaysRun.run(); //run the always run

        Conditional condition = currentConditional < endConditions.size() ? endConditions.get(currentConditional) : Conditional.DEFAULT;//get current conditional

        if (!initializedConditional){
            condition.init();
            condition.setupStates(condition.linkedStates);
            initializedConditional = true;
        }

        condition.execute(); //execute condition

        if (condition.linkedStates != null && condition.linkedStates.length> 0) {
            for (int idxCall : condition.linkedStates) { //executes currently linked states
                if (idxCall > -1) states.get(idxCall).run();
            }
        }

        if (condition.isFinished()){ //change conditions if needed
            currentConditional++;
            initializedConditional = false;
            condition.end();
        }
    }

    public void reset(){
        currentConditional = 0;
    }
}
