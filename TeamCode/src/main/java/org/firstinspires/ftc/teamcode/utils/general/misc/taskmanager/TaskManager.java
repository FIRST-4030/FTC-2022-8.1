package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager;

import java.util.Arrays;
import java.util.Vector;

public class TaskManager {

    public int currentConditional;

    public Vector<Conditional> endConditions;
    public Runnable alwaysRun;
    public Vector<Runnable> states;

    public TaskManager(){
        endConditions = new Vector<>();
        states = new Vector<>();

        alwaysRun = () -> {};

        currentConditional = 0;
    }

    public void addConditions(Conditional... conditionals){
        for (Conditional c: conditionals) {
            c.init();
            endConditions.add(c);
        }
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
        condition.check(); //check if the condition is met

        if (condition.status == Conditional.STATUS.PASSED){ //change conditions if needed
            currentConditional++;
            condition = currentConditional < endConditions.size() ? endConditions.get(currentConditional) : Conditional.DEFAULT;
        }

        if (condition.linkedStates != null && condition.linkedStates.length> 0) {
            for (int idxCall : condition.linkedStates) { //executes currently linked states
                if (idxCall > -1) states.get(idxCall).run();
            }
        }
    }

    public void resetIdx(){
        currentConditional = 0;
    }
}
