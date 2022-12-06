package org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.FileRWRow;

import java.util.List;

public abstract class LexerTemplate {

    private int x_position = 0;
    private int y_position = 0;
    private List<LexerToken> tokenBuffer;

    public abstract List<LexerToken> lex(List<FileRWRow> rows);

    protected void incrX(){
        x_position += 1;
    }

    protected void decrX(){
        x_position -= 1;
    }

    protected void setX(int nX){
        x_position = nX;
    }

    protected int getX(){
        return x_position;
    }

    protected void incrY(){
        y_position += 1;
    }

    protected void decrY(){
        y_position -= 1;
    }

    protected void setY(int nY){
        y_position = nY;
    }

    protected int getY(){
        return y_position;
    }

    protected void setTokenBuffer(List<LexerToken> nTokens){this.tokenBuffer = nTokens;}

    protected List<LexerToken> getTokenBuffer(){return this.tokenBuffer;}
}
