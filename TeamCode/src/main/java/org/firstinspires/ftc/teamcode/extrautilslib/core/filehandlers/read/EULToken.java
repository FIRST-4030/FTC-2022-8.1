package org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.read;

public class EULToken {

    public static final EULToken numTemplate = new EULToken("NUMBER", "");
    public static final EULToken intTemplate = new EULToken("INTEGER", "");
    public static final EULToken doubleTemplate = new EULToken("DOUBLE", "");
    public static final EULToken stringTemplate = new EULToken("STRING", "");
    public static final EULToken errorTemplate = new EULToken("ERROR", "");
    public static final EULToken eofTemplate = new EULToken("EOF", "");


    private String type;
    private String lexeme;
    private int x, y;

    public EULToken(){
        this.type = "";
        this.lexeme = "";
        this.x = 0;
        this.y = 0;
    }

    public EULToken(String type, String lexeme){
        this.type = type;
        this.lexeme = lexeme;
        this.x = 0;
        this.y = 0;
    }

    public EULToken(String type, String lexeme, int x, int y){
        this.type = type;
        this.lexeme = lexeme;
        this.x = x;
        this.y = y;
    }

    public EULToken copy(){
        return new EULToken(this.type, this.lexeme, this.x, this.y);
    }

    public EULToken setType(String nType){
        type = nType;
        return this;
    }

    public String getType(){
        return type;
    }



    public EULToken setLexeme(String nLexeme){
        lexeme = nLexeme;
        return this;
    }

    public String getLexeme(){
        return lexeme;
    }



    public EULToken setX(int nX){
        this.x = nX;
        return this;
    }

    public int getX(){
        return this.x;
    }

    public EULToken setY(int nY){
        this.y = nY;
        return this;
    }

    public int getY(){
        return this.y;
    }

}
