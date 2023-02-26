package org.firstinspires.ftc.teamcode.utils.fileRW.main.writer;

import java.util.ArrayList;
import java.util.List;

public abstract class WriterTemplate {

    private List<List<WritableElement>> bufferedElements;

    public abstract WritableElement translateToElement(Object element);

    public abstract List<WritableElement> translateToRow(List<?> elementList);

    public void writeToRow(int row, List<WritableElement> writableElements){
        for (int i = this.bufferedElements.size() - 1; i < row; i++){
            this.bufferedElements.add(new ArrayList<>());
        }
        this.bufferedElements.get(row).addAll(writableElements);
    }

    public void writeToCoord(int x, int y, WritableElement writableElement){
        for (int i = this.bufferedElements.size() - 1; i < y; i++){
            this.bufferedElements.add(new ArrayList<>());
        }

        for (int i = this.bufferedElements.get(y).size() - 1; i < x; i++){
            this.bufferedElements.add(new ArrayList<>());
        }

        this.bufferedElements.get(y).add(x, writableElement);
    }

    public void deleteElement(int x, int y){
        this.bufferedElements.get(y).remove(x);
    }

    public void deleteRow(int row){
        this.bufferedElements.remove(row);
    }

    public void deleteAll(){
        this.bufferedElements.clear();
    }

    public void setBufferedElements(List<List<WritableElement>> nBufferedElements){
        this.bufferedElements = nBufferedElements;
    }

    public List<List<WritableElement>> getBufferedElements(){
        return this.bufferedElements;
    }

    public abstract List<String> getBufferAsStringRows();
}
