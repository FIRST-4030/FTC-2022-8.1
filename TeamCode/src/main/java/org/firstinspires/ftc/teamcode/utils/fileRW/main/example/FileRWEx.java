package org.firstinspires.ftc.teamcode.utils.fileRW.main.example;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.FileRW;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.csv.CSVLexer;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv.CSVParsedFile;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv.CSVParser;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.csv.CSVWriter;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

import java.util.Arrays;

public class FileRWEx extends LoopUtil {

    @Override
    public void opInit() {
        FileRW frw = new FileRW(FileRW.defaultDir, new CSVLexer(), new CSVParser(), new CSVWriter()); //construct FileRW
        frw.init(); //initialize so we don't get a null directory

        frw.writeToRow(0, frw.translateToRow(Arrays.asList(10.5, 3.14e-5, 69.024))); //translate an array in WritableElements and pass that into the write to row method
        frw.finalizeWriteTo("placeholder.virus");

        CSVParsedFile csvParsedFile = null;
        try {
            csvParsedFile = (CSVParsedFile) frw.parse(frw.read("placeholder.virus")); //reads a file, then passes it to a parser that returns a parsed file for us to inquire information from
        } catch (Exception e){
            telemetry.log().add(e.getStackTrace().toString());
        }

        //check if the ParsedFile is not null since the methods depend on the data being there in the first place
        if (csvParsedFile != null){
            csvParsedFile.getRow(0); //returns the first row, but the return is ignored for this example...
        }
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
