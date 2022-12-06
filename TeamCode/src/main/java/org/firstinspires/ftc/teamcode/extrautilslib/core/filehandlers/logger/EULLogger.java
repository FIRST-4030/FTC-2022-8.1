package org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.logger;

import java.util.ArrayList;
import java.util.HashMap;

public interface EULLogger<T> {
    String toString();
    HashMap<T, ArrayList<String>> getLog();
    void log(T level, String msg);
}
