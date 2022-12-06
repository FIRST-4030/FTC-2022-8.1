package org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerToken;

public class CSVToken implements LexerToken {

    //pre-defined token types are easier to copy, mod, and append w/o remembering what each token needs.
    public static final CSVToken stringTemplate = new CSVToken("STRING", "", 0, 0);
    public static final CSVToken intTemplate = new CSVToken("INT", "", 0, 0);
    public static final CSVToken doubleTemplate = new CSVToken("DOUBLE", "", 0, 0);
    public static final CSVToken errorTemplate = new CSVToken("ERROR", "", 0, 0);
    public static final CSVToken boolTemplate = new CSVToken("BOOL", "", 0, 0);
    public static final CSVToken eofTemplate = new CSVToken("EOF", "", 0, 0);

    private String content, type;
    private int x, y;

    public CSVToken(String type, String content, int x, int y){
        this.type = type;
        this.content = content;
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString(){
        return "Col: " + this.x + " |Row: " + this.y + " |Type: " + this.type + " |Content: " + this.content;
    }

    public CSVToken copy(){return new CSVToken(this.type, this.content, this.x, this.y);}

    public String getType(){return this.type.toString();}
    public void setType(String type){this.type = type;}

    public String getContent(){return this.content;}
    public void setContent(String content){this.content = content;}

    public int getX(){return this.x;}
    public void setX(int x){this.x = x;}

    public int getY(){return this.y;}
    public void setY(int y){this.y = y;}
}