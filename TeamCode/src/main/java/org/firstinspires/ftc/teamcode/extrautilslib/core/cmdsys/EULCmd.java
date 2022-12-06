package org.firstinspires.ftc.teamcode.extrautilslib.core.cmdsys;

import java.util.Arrays;

public class EULCmd {

    private String content = "";
    private int argNumber;
    private EULArgType[] types;

    public EULCmd(){
        this.argNumber = 0;
        this.types = new EULArgType[0];
    }

    public EULCmd(int argNumber){
        this.argNumber = argNumber;
        this.types = new EULArgType[argNumber];
    }

    public EULCmd(EULArgType... types){
        this.argNumber = types.length;
        this.types = types;
    }

    public int getArgNumber(){
        return this.argNumber;
    }

    public EULArgType getArgType(int i){
        return this.types[i];
    }

    public EULCmd replaceArgType(EULArgType... types){
        this.types = types;
        this.argNumber = this.types.length;

        return this;
    }

    public EULCmd setContent(String nContent){
        this.content = nContent;
        return this;
    }

    public String getContent(){
        return this.content;
    }

    @Override
    public String toString(){
       return " Content: " + content + " Argument Number: " + argNumber + " Argument Type(s): " + Arrays.toString(types).substring(0, Arrays.toString(types).length() - 1);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        EULCmd eulCmd = (EULCmd) o;

        if (argNumber != eulCmd.argNumber) return false;
        // Probably incorrect - comparing Object[] arrays with Arrays.equals
        return Arrays.equals(types, eulCmd.types);
    }

    @Override
    public int hashCode() {
        int result = argNumber;
        result = 31 * result + Arrays.hashCode(types);
        return result;
    }
}