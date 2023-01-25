package org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

public class XMLRW {

    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/ROBOT_XML_DATA/";
    private static boolean initialized = false;

    private XMLRW(){}

    public static void init(){
        if (!initialized) {
            File directory = new File(defaultDir);
            if (!directory.exists()) {
                directory.mkdir();
            }
            initialized = true;
        }
    }
}
