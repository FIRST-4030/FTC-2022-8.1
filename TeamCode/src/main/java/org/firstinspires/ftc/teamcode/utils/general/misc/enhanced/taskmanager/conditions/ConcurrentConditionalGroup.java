package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

public class ConcurrentConditionalGroup extends Conditional{

    private Conditional[] conditionals = new Conditional[0];
    private boolean[] finished = new boolean[0];
    private Runnable[] states = new Runnable[0];

    public ConcurrentConditionalGroup(Conditional... conditionals){
        if (conditionals != null) {
            this.conditionals = conditionals;
            this.finished = new boolean[conditionals.length];
        }
    }

    @Override
    public void bindManager(ConditionalManager manager){
        this.states = manager.getStates().toArray(new Runnable[0]);
    }

    @Override
    public void init() {
        for (Conditional c: conditionals) {
            c.init();
        }
    }

    @Override
    public void setupStates(int[] linkedStates) {

    }

    @Override
    public void execute() {
        for (int i = 0; i < conditionals.length; i++) {
            if (!conditionals[i].isFinished()) {
                conditionals[i].execute();
                for (int n : conditionals[i].linkedStates) {
                    states[n].run();
                }
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
