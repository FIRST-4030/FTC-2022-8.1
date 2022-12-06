package org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerToken;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.csv.CSVToken;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParserTemplate;

import java.io.IOException;
import java.util.List;
import java.util.Vector;

public class CSVParser extends ParserTemplate {

    @Override
    public ParsedFile parse(List<LexerToken> lexerTokens) throws IOException {

        boolean matchType = lexerTokens.get(0) instanceof CSVToken;

        if (!matchType){throw new IOException("Lexer Token classes cannot be parsed!");}

        Vector<CSVToken> csvTokens = new Vector<>();
        CSVParsedFile output;

        //map the tokens
        for (int i = 0; i < lexerTokens.size(); i++) {
            csvTokens.add((CSVToken) lexerTokens.get(i));
        }

        int parsedMapLen = csvTokens.get(csvTokens.size() - 1).getY();
        Vector<Object>[] parsedMap = new Vector[parsedMapLen];

        String strTokenType = CSVToken.stringTemplate.getType();
        String intTokenType = CSVToken.intTemplate.getType();
        String doubleTokenType = CSVToken.doubleTemplate.getType();
        String errorTokenType = CSVToken.errorTemplate.getType();
        String tokenType;

        CSVToken csvToken;

        for (int i = 0; i < parsedMap.length; i++){
            parsedMap[i] = new Vector<>();
        }

        for (int t = 0; t < csvTokens.size() - 1; t++) {
            csvToken = csvTokens.get(t);
            tokenType = csvToken.getType();
            if (tokenType == strTokenType){
                parsedMap[csvToken.getY()].add(csvToken.getContent());
            } else if (tokenType == intTokenType) {
                parsedMap[csvToken.getY()].add(Integer.parseInt(csvToken.getContent()));
            } else if (tokenType == doubleTokenType) {
                parsedMap[csvToken.getY()].add(Double.parseDouble(csvToken.getContent()));
            } else {
                parsedMap[csvToken.getY()].add(csvToken.getContent());
            }
        }

        (output = new CSVParsedFile()).setContentMap(parsedMap);
        return output;
    }
}