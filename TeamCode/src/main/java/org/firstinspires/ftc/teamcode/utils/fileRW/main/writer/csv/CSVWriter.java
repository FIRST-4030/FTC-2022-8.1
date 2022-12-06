package org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WritableElement;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WriterTemplate;

import java.util.List;
import java.util.Vector;

public class CSVWriter extends WriterTemplate {

    public CSVWriter(){
        setBufferedElements(new Vector<>());
    }

    @Override
    public WritableElement translateToElement(Object element) {
        return new CSVElement(element);
    }

    @Override
    public List<WritableElement> translateToRow(List<?> elementList) {
        Vector<WritableElement> output = new Vector<>();
        for (Object element: elementList) {
            output.add(new CSVElement(element));
        }
        return output;
    }

    @Override
    public List<String> getBufferAsStringRows() {
        List<String> output = new Vector<>();
        List<List<WritableElement>> buffer = getBufferedElements();
        String row;
        for (List<WritableElement> vector: buffer) {
            row = "";
            for (WritableElement ele: vector) {
                row += ele.toString();
            }
            output.add(row);
        }

        return output;
    }
}
