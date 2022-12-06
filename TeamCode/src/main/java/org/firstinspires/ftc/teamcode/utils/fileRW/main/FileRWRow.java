package org.firstinspires.ftc.teamcode.utils.fileRW.main;

public class FileRWRow {

    private String row;
    private int length;

    public FileRWRow(){
        this.row = "";
        updateCap();
    }

    public FileRWRow(String initial_data){
        this.row = initial_data;
        updateCap();
    }

    private void updateCap(){
        this.length = this.row.length();
    }

    public void append(String data){
        this.row += data;
        updateCap();
    }

    public void insert(int index, String data){
        String first_half = this.row.substring(0, index);
        String second_half = this.row.substring(index, length);
        this.row = first_half + data + second_half;
        updateCap();
    }

    public void remove(int begin_idx, int end_idx){
        String first_half = this.row.substring(0, begin_idx);
        String second_half = this.row.substring(end_idx, length);
        this.row = first_half + second_half;
        updateCap();
    }

    @Override
    public String toString(){
        return this.row;
    }
}