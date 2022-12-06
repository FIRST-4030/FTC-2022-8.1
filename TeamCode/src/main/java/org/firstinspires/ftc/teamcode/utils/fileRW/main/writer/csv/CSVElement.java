package org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WritableElement;

public class CSVElement implements WritableElement {

    private Object content;

    public CSVElement(){
        this.content = "";
    }

    public CSVElement(Object data){
        this.content = data;
    }

    public String toString(){
        return this.content.toString() + ", ";
    }
}
