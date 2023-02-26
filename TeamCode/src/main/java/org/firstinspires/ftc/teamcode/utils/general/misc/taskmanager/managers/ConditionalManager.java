package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.managers;

import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions.Conditional;
import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions.ParallelConditionalGroup;

import java.util.Arrays;
import java.util.Stack;
import java.util.Vector;

public class ConditionalManager implements Runnable{

    private Runnable alwaysRun;

    private Stack<Conditional> conditionalStack;
    private Conditional currentConditional;

    private Vector<Runnable> states;


    private boolean initFlag = true,
                    usesStatesFlag;

    /**
     * Conditional Manager is a more flexible version of TaskManager
     * <br>This machine can function as a state machine, or as a sequential executor for a more command-like style
     */
    public ConditionalManager(){
        alwaysRun = () -> {};

        conditionalStack = new Stack<>();
        currentConditional = null;
    }

    public ConditionalManager addConditional(Conditional conditional){
        this.conditionalStack.insertElementAt(conditional, 0);
        return this;
    }

    public ConditionalManager addConditionals(Conditional... conditionals){
        for (Conditional c: conditionals) this.conditionalStack.insertElementAt(c, 0);
        return this;
    }

    public ConditionalManager removeConditional(int idx){
        this.conditionalStack.remove(this.conditionalStack.size() - 1 - idx);
        return this;
    }

    public ConditionalManager removeConditionals(int... idx){
        int[] sorted = Arrays.copyOf(idx, idx.length);
        Arrays.sort(sorted); //sort in ascending order to later remove back to front

        for (int i = 0; i < sorted.length; i++) {
            this.conditionalStack.remove(sorted[sorted.length - 1 - i]); //remove back to front
        }

        return this;
    }

    public Stack<Conditional> getConditionalStack(){
        return conditionalStack;
    }

    public ConditionalManager addStates(Runnable... states){
        this.states.addAll(Arrays.asList(states)); //add all the necessary states
        return this;
    }

    public ConditionalManager addState(Runnable state, int idx){
        states.add(idx, state);
        return this;
    }

    public ConditionalManager removeStates(int... idx){
        int[] sorted = Arrays.copyOf(idx, idx.length);
        Arrays.sort(sorted); //sort in ascending order to later remove back to front

        for (int i = 0; i < sorted.length; i++) {
            states.remove(sorted[sorted.length - 1 - i]); //remove back to front
        }
        return this;
    }

    public Vector<Runnable> getStates(){
        return states;
    }

    /**
     * This method is the only method you need to run in the main loop to proceed through the state machine command stack hybrid
     */
    public void execute(){
        alwaysRun.run();

        if (!conditionalStack.empty()){
            if (initFlag) {
                currentConditional = conditionalStack.peek();
                currentConditional.init();
                currentConditional.setupStates(currentConditional.linkedStates);

                if(currentConditional.getClass().getSimpleName().equals(ParallelConditionalGroup.class.getSimpleName())){
                    ParallelConditionalGroup group = (ParallelConditionalGroup) currentConditional;
                    group.bindManager(this);
                    currentConditional = group; //probably redundant
                }

                initFlag = false;
                usesStatesFlag = currentConditional.linkedStates != null && currentConditional.linkedStates.length > 0;;
            }

            currentConditional.execute();

            if (usesStatesFlag && currentConditional.linkedStates != null && currentConditional.linkedStates.length > 0){
                for (int n: currentConditional.linkedStates) {
                    if (n >= 0 && n < states.size()) states.get(n).run();
                }
            }

            if (currentConditional.isFinished()){
                currentConditional.end();
                conditionalStack.pop();

                initFlag = true;
            }
        }
    }

    @Override
    public void run() {
        while (!conditionalStack.empty()){
            this.execute();
        }
    }
}
