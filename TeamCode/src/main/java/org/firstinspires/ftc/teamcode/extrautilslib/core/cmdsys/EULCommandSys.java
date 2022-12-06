package org.firstinspires.ftc.teamcode.extrautilslib.core.cmdsys;

import org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.read.EULToken;
import org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.read.EULTokenizer;
import com.sun.tools.javac.util.Pair;

import java.util.HashMap;

public class EULCommandSys {

    protected static EULTokenizer tokenizer = new EULTokenizer();
    protected String cmdPrefix; //define prefix
    protected char cmdSeparator;

    protected HashMap<String, EULCmd[]> cmdMap; //define what command maps to what

    public EULCommandSys(String prefix){
        this.cmdPrefix = prefix.trim();
        this.cmdSeparator = ':';
    }

    public EULCommandSys(String prefix, char separator){
        this.cmdPrefix = prefix.trim();
        this.cmdSeparator = separator;
    }

    public Pair parseCmd(String input){
        String trimmedIn = input.trim();
        String prefix;
        String cmd, args;
        int argStart, separatorPos;

        Pair output = null;

        if (trimmedIn.length() > (cmdPrefix.length() + 1)){

            if (cmdPrefix.equals(prefix = trimmedIn.substring(0, cmdPrefix.length()))){
                argStart = prefix.length();
                if (trimmedIn.indexOf(cmdSeparator) != -1) {
                    cmd = trimmedIn.substring(argStart, separatorPos = trimmedIn.indexOf(cmdSeparator));

                    args = trimmedIn.substring(separatorPos + 1).trim();

                    EULCmd r = new EULCmd();
                    r.setContent(args);
                    String[] toLex = args.split(" ");
                    EULArgType[] argTypes = new EULArgType[toLex.length];

                    for (int i = 0; i < toLex.length; i++){
                        EULToken token = tokenizer.classify(toLex[i], 0, 0);

                        switch (token.getType()){
                            case "INTEGER":
                                argTypes[i] = EULArgType.INTEGER;
                                break;
                            case "DOUBLE":
                                argTypes[i] = EULArgType.DOUBLE;
                                break;
                            case "STRING":
                                argTypes[i] = EULArgType.STRING;
                                break;
                        }
                    }

                    output = new Pair<>(cmd, (new EULCmd(argTypes)).setContent(args));

                } else {
                    output = new Pair<>(trimmedIn.substring(argStart), new EULCmd());
                }
            }
        }

        return output;
    }
}
