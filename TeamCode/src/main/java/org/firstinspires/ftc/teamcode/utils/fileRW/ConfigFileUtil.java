package org.firstinspires.ftc.teamcode.utils.fileRW;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.List;

/**
 * Utility functions for log files.
 * Can only read and write in .csv right now
 */
public class ConfigFileUtil {
    public static final String dir = Environment.getExternalStorageDirectory().getPath() + "/UD_ROBOT_CONFIG";
    public static final String fileFormat = ".virus";
    private static boolean initialized = false;

    public static void init(){
        if (initialized) return;
        initialized = true;
        File directory = new File(dir);
        if(!directory.exists()) {
            directory.mkdir();
        }
    }

    public enum ConfigDataType{
        INT,
        FLOAT,
        STRING,
        DOUBLE,
        BYTE,
        LONG
    }

    //private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    public static void writeToConfig(String name, List<?> data, int rows) {
        if (!initialized) throw new IllegalStateException("You did not initialize this class! Try calling 'ConfigFileUtil.init();'!");
        try {

            PrintWriter writer = new PrintWriter( dir + "/" + name + fileFormat, "UTF-8");
            BufferedWriter bufferedWriter = new BufferedWriter(writer);
            for (int i = 0; i < rows; i++) {
                String line = "";
                for (int j = 0; j < data.size()/rows; j++) {
                    line += data.get(i * rows + j).toString() + ",";
                }

                bufferedWriter.write(line);
                bufferedWriter.newLine();
            }

            bufferedWriter.flush();
            bufferedWriter.close();

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static File getConfig(String name){
        if (!initialized) throw new IllegalStateException("You did not initialize this class! Try calling 'ConfigFileUtil.init();'!");
        return new File(dir + "/" + name + fileFormat);
    }

    public static void readConfig(String name, Object[][] target, ConfigDataType type){
        if (!initialized) throw new IllegalStateException("You did not initialize this class! Try calling 'ConfigFileUtil.init();'!");
        int rows = target.length;
        String line;
        String[] data;

        try{
            FileReader fileReader = new FileReader(ConfigFileUtil.getConfig(name));
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            for (int i = 0; i < rows; i++) {
                if ((line = bufferedReader.readLine()) != null){
                    data = line.split(",");

                    for (int j = 0; j < data.length; j++){
                        switch (type){
                            case INT:
                                target[i][j] = Integer.parseInt(data[j]);
                                break;
                            case BYTE:
                                target[i][j] = Byte.parseByte(data[j]);
                                break;
                            case FLOAT:
                                target[i][j] = Float.parseFloat(data[j]);
                                break;
                            case LONG:
                                target[i][j] = Long.parseLong(data[j]);
                                break;
                            case DOUBLE:
                                target[i][j] = Double.parseDouble(data[j]);
                                break;
                            case STRING:
                                target[i][j] = data[j];
                                break;
                        }
                    }

                }
            }

        } catch (Exception e){
            e.printStackTrace();
        }
    }
}
