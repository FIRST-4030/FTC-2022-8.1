package org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.conditions;

import org.firstinspires.ftc.teamcode.utils.general.misc.taskmanager.managers.ConditionalManager;

public class ParallelConditionalGroup extends Conditional{

    private ConcurrentConditionalGroup parallelizedConcurrentConditionals;
    private ConditionalManager manager;

    public ParallelConditionalGroup(Conditional... conditionals){
        this(false, conditionals);
    }

    public ParallelConditionalGroup(boolean dupeStatesFlag, Conditional... conditionals){
        this.parallelizedConcurrentConditionals = new ConcurrentConditionalGroup(dupeStatesFlag, conditionals);
    }

    @Override
    public void bindManager(ConditionalManager manager){
        this.manager = new ConditionalManager();
        this.manager.addStates(manager.getStates().toArray(new Runnable[0]));
        this.manager.addConditional(parallelizedConcurrentConditionals);
    }

    @Override
    public void init() {
        new Thread(this.manager).start();
    }

    @Override
    public void setupStates(int[] linkedStates) {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
