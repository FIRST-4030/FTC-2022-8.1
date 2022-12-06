package org.firstinspires.ftc.teamcode.utils.threadingUtils;

public abstract class Background implements Runnable{
    private boolean running = false;
    private Thread thread;

    public void start() {
        if (thread != null) return;
        running = true;
        thread = new Thread(this);
        thread.start();
    }

    public void stop() {
        if (thread == null) return;
        running = false;
        thread.interrupt();
    }

    @Override
    public void run() {
        while (running) {
            loop();
        }
    }

    public abstract void loop();
}
