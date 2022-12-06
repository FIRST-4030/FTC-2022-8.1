package org.firstinspires.ftc.teamcode.utils.fileRW.main;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParserTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WritableElement;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WriterTemplate;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Vector;

public class FileRW{

    /**
     * This is usually the default directory used for reading from and writing to so we don't have to type out the entire path each time
     */
    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/UD_ROBOT_CONFIG/";

    private String root_directory; //saves us time since it appends the root directory so we can add the relative path from that as our read inputs
    private boolean initialized = false; //init status

    private LexerTemplate lexer;
    private ParserTemplate parser;
    private WriterTemplate writer;

    //explicitly removes empty constructor
    private FileRW(){}

    /**
     * almost self-explaining, but lexerBehavior, parserBehavior, writeBehavior all require their respective objects passed in,<br>
     * like CSV: new FileRW([placeholder], new CSVLexer(), new CSVParser(), new CSVWriter());
     * @param root_directory
     * @param lexerBehavior
     * @param parserBehavior
     * @param writeBehavior
     */
    public FileRW(String root_directory, LexerTemplate lexerBehavior, ParserTemplate parserBehavior, WriterTemplate writeBehavior){
        this.root_directory = root_directory;
        this.lexer = lexerBehavior;
        this.parser = parserBehavior;
        this.writer = writeBehavior;
    }

    /**
     * almost self-explaining, but lexerBehavior, parserBehavior, writeBehavior all require their respective objects passed in,<br>
     * like CSV: new FileRW(new CSVLexer(), new CSVParser(), new CSVWriter());
     * @param lexerBehavior
     * @param parserBehavior
     * @param writeBehavior
     */
    public FileRW(LexerTemplate lexerBehavior, ParserTemplate parserBehavior, WriterTemplate writeBehavior){
        this.root_directory = FileRW.defaultDir;
        this.lexer = lexerBehavior;
        this.parser = parserBehavior;
        this.writer = writeBehavior;
    }

    /**
     * This method needs to be called manually ~~because I'm lazy~~ because it's good practise I guess <br>
     * The method's function is to check if the directory exists, and if not, then create it, which solves the null file problem
     */
    public void init(){
        File directory = new File(root_directory);
        if (!directory.exists()){
            directory.mkdir();
        }

        initialized = true;
    }

    /**
     * This method returns a Vector of FileRWRow for <br>
     * (1) Thread Safety <br>
     * (2) For further lexing then parsing
     * @param filepath
     * @return
     */
    public Vector<FileRWRow> read(String filepath) {
        File readable = new File(root_directory + filepath);

        if (!initialized) throw new RuntimeException("This class hasn't been initialized yet!");

        try {
            BufferedReader br = new BufferedReader(new FileReader(readable));

            Vector<FileRWRow> output = new Vector<>();

            String line = "";
            while (line != null){
                line = br.readLine();
                output.add(new FileRWRow(line));
            }

            br.close(); //explicitly call close method

            return output;
        } catch (Exception e){
            e.printStackTrace();
        }

        return null;
    }

    /**
     * This method should be called after finishing a write (just like a BufferedWriter's flush method) to do all buffered write commands at once
     * @param filepath
     */
    public void finalizeWriteTo(String filepath){
        File writable = new File(root_directory + filepath);

        if (!initialized) throw new RuntimeException("This class hasn't been initialized yet!");

        try{
            BufferedWriter bw = new BufferedWriter(new FileWriter(writable));
            List<String> strings = writer.getBufferAsStringRows();

            for (String s: strings) {
                bw.write(s);
                bw.newLine();
            }

            bw.flush();

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    //Wrapper methods for handling the write buffer

    /**
     * This method will return a parsed file, which have simple methods to query information about the file, like an element list in certain rows (CSVParsedFile)
     * @param rows
     * @return ParsedFile
     * @throws IOException
     */
    public ParsedFile parse(Vector<FileRWRow> rows) throws IOException {
        return parser.parse(lexer.lex(rows));
    }

    /**
     * When this method is called, it expects a row number and data passed to it to buffer a write to that row of a file
     * @param row
     * @param writableElements
     */
    public void writeToRow(int row, List<WritableElement> writableElements){
        writer.writeToRow(row, writableElements);
    }

    /**
     * Same stuff as writeToRow, but this time, it buffers a write to a single element at the specified x(col) and y(row)
     * @param x
     * @param y
     * @param writableElement
     */
    public void writeToCoord(int x, int y, WritableElement writableElement){
        writer.writeToCoord(x, y, writableElement);
    }

    /**
     * Deletion of an element at the specified coord (x(col), y(row)) will be buffered
     * @param x
     * @param y
     */
    public void deleteElement(int x, int y){
        writer.deleteElement(x, y);
    }

    /**
     * Removes a row from the buffer
     * @param row
     */
    public void deleteRow(int row){
        writer.deleteRow(row);
    }

    /**
     * Purges the buffer without a flush
     */
    public void deleteAll(){
        writer.deleteAll();
    }

    /**
     * Translates an element into a format the writer can read
     * @param element
     * @return WritableElement
     */
    public WritableElement translateToElement(Object element) {
        return writer.translateToElement(element);
    }

    /**
     * Iterates over every object in the list passed and translates them into a format the writer can read
     * @param elementList
     * @return WritableElements
     */
    public List<WritableElement> translateToRow(List<?> elementList) {
        return writer.translateToRow(elementList);
    }

    /**
     * Wrapper method for setting the buffer if you have a pre-made 2d expandable array written
     * @param nBufferedElements
     */
    public void setBufferedElements(List<List<WritableElement>> nBufferedElements){
        writer.setBufferedElements(nBufferedElements);
    }

    /**
     * Wrapper method for getting the current buffered writes
     * @return bufferedElements
     */
    public List<List<WritableElement>> getBufferedElements(){
        return writer.getBufferedElements();
    }


}
