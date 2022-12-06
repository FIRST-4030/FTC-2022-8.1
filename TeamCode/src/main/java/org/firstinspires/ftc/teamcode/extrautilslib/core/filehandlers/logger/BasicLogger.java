package org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.logger;


import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.extrautilslib.core.timer.EULClock;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.*;

public class BasicLogger implements EULLogger<Level>{

    private HashMap<Level, ArrayList<String>> logMap;
    private final boolean logTime;
    private final Handler handler;
    private EULClock clock;

    public BasicLogger(Handler handler, boolean logTime) {

        this.logMap = new HashMap<>();
        this.logTime = logTime;

        this.clock = new EULClock();
        this.clock.start();
        this.handler = handler;
        this.handler.setLevel(Level.ALL);
    }

    @Override
    public HashMap getLog() {
        return this.logMap;
    }

    @Override
    public void log(Level level, String msg) {
        double time = this.clock.getElapsedTime() * EULConstants.NANO2SEC;
        String header = "";
        header += logTime ? "{Elapsed: " + time + " seconds}: " : "";


        if (this.logMap.containsKey(level)){
            this.logMap.get(level).add(header + msg);
        } else {
            this.logMap.put(level, new ArrayList<>());
            log(level, msg);
        }
    }

    public void outputAll(){
        Level[] levels = this.logMap.keySet().toArray(new Level[0]);
        ArrayList<LogRecord> records = new ArrayList<>();
        int len = levels.length;

        for (int i = 0; i < len; i++) {
            Level currentLevel = levels[i];
            ArrayList<String> msgList = this.logMap.get(currentLevel);
            for (String s: msgList) {
                records.add(new LogRecord(currentLevel, s));
            }
        }

        for (LogRecord log: records) {
            this.handler.publish(log);
        }
    }

    public void setLogLevel(Level level){
        this.handler.setLevel(level);
    }
}
