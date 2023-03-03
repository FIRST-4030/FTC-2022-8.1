package org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.conditions;

import org.firstinspires.ftc.teamcode.utils.general.misc.enhanced.taskmanager.managers.ConditionalManager;

public class ParallelConditionalGroup extends Conditional{

    private ConcurrentConditionalGroup parallelizedConcurrentConditionals;
    private ConditionalManager manager;

    public ParallelConditionalGroup(Conditional... conditionals){
        this.parallelizedConcurrentConditionals = new ConcurrentConditionalGroup(conditionals);
    }

    @Override
    public void bindManager(ConditionalManager manager){
        this.manager = new ConditionalManager();
        this.parallelizedConcurrentConditionals.bindManager(manager);
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
