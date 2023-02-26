package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions;

import java.util.HashSet;
import java.util.Vector;

public class ConcurrentConditionalGroup extends Conditional{

    private Conditional[] conditionals = new Conditional[0];
    private boolean[] finished = new boolean[0];
    private boolean dupeStatesFlag;

    public ConcurrentConditionalGroup(Conditional... conditionals){
        this(false, conditionals);
    }

    public ConcurrentConditionalGroup(boolean dupeStatesFlag, Conditional... conditionals){
        this.dupeStatesFlag = dupeStatesFlag;
        if (conditionals != null) {
            this.conditionals = conditionals;
            this.finished = new boolean[conditionals.length];
        }
    }

    @Override
    public void init() {
        for (Conditional c: conditionals) {
            c.init();
        }
    }

    @Override
    public void setupStates(int[] linkedStates) {
        Vector<Integer> statesToRun = new Vector<>();
        for (Conditional c: conditionals) {
            int[] stateList = new int[0];
            c.setupStates(stateList);
            for (int n: stateList) {
                statesToRun.add(n);
            }
        }

        if (dupeStatesFlag){
            linkedStates = new int[statesToRun.size()];
            for (int i = 0; i < linkedStates.length; i++) {
                linkedStates[i] = statesToRun.get(i);
            }
        } else {
            Integer[] setArr = (new HashSet<>(statesToRun)).toArray(new Integer[0]);
            linkedStates = new int[setArr.length];
            for (int i = 0; i < linkedStates.length; i++){
                linkedStates[i] = setArr[i];
            }
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < conditionals.length; i++) {
            if (!conditionals[i].isFinished()) {
                conditionals[i].execute();
            } else {
                if (!finished[i]) {
                    conditionals[i].end();
                    finished[i] = true;
                }
            }
        }
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        int finishedCount = 0;
        for (boolean b : finished)
            if (b) finishedCount++;
        return finishedCount == finished.length;
    }
}
