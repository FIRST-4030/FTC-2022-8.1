package org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;

import java.util.Vector;

public class CSVParsedFile implements ParsedFile {

    private Vector<?>[] contentMap;

    public CSVParsedFile(){
        this.contentMap = null;
    }

    public void setContentMap(Vector<?>[] contentMap){
        this.contentMap = contentMap;
    }

    public Object getElement(int x, int y){
        if (!(y < contentMap.length)) throw new ArrayIndexOutOfBoundsException("This row doesn't exist!");
        if (!(x < contentMap[y].size())) throw new ArrayIndexOutOfBoundsException("This column doesn't exist!");
        return this.contentMap[y].get(x);
    }

    public Vector<?> getRow(int row){
        if (!(row < contentMap.length)) throw new ArrayIndexOutOfBoundsException("This row doesn't exist!");
        return this.contentMap[row];
    }

    @Override
    public void delete() {
        for (Vector<?> row: contentMap) {
            row.clear();
        }
        contentMap = null;
    }

    @Override
    public Vector<?>[] getAll() {
        return this.contentMap;
    }
}
