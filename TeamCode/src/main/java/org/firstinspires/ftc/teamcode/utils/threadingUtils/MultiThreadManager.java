package org.firstinspires.ftc.teamcode.utils.threadingUtils;

public class MultiThreadManager {

    private MRUTask[] mruTasks;
    private boolean[] isDoneArray;
    private Thread[] mruThreads;

    private boolean started = false;

    public MultiThreadManager(MRUTask... mruTasks){
        this.mruTasks = new MRUTask[mruTasks.length]; //initialize task list
        this.mruThreads = new Thread[mruTasks.length]; //initialize threads

    }

    public void init(){
        if (!started){
            for (int i = 0; i < mruTasks.length; i++) {
                this.mruThreads[i] = new Thread(this.mruTasks[i]);
            }
        }
    }

    public void start(){
        if (!started){
            for (int i = 0; i < mruTasks.length; i++) {
                this.mruThreads[i].start();
            }
        }
    }

    public synchronized void stop(){
        if (started){
            for (int i = 0; i < mruTasks.length; i++) {
                try {
                    this.mruThreads[i].join();
                } catch (Exception e){
                    e.printStackTrace();
                }
            }
        }
    }
}