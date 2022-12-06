package org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.csv;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.FileRWRow;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerToken;

import java.util.List;
import java.util.Vector;

public class CSVLexer extends LexerTemplate {

    /*
    //pre-defined token types are easier to copy, mod, and append w/o remembering what each token needs.
    private final CSVToken stringTemplate = new CSVToken("STRING", "", 0, 0);
    private final CSVToken intTemplate = new CSVToken("INT", "", 0, 0);
    private final CSVToken doubleTemplate = new CSVToken("DOUBLE", "", 0, 0);
    private final CSVToken errorTemplate = new CSVToken("ERROR", "", 0, 0);

     */

    //pre-defined set of characters are nice to have. Not a definitive list of them though.
    private final static String
            CSV_DIGITS = "0123456789",
            CSV_LETTERS = "AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz",
            CSV_SYMBOLS = "~`!@#$%^&*-_+=(){}[]|\\:;\"'<,>.?/";

    //some buffers are here, like the token buffer, the toLex (can make classify a no input func, but that reduces flexibility)
    private String toLex = "";
    //private List<LexerToken> tokenBuffer;

    //defined whitespace here because it's annoying to remember the difference between " " and ' ' when comparing chars
    private final char charWhitespace = ' ';
    private final String stringWhitespace = " ";

    public CSVLexer(){
        setTokenBuffer(new Vector<>());
    }

    //non-static because we want to be able to have multiple instances of this class; same reason for below methods
    public List<LexerToken> lex(List<FileRWRow> fileLines){
        setX(0);
        setY(0);
        for (int i = 0; i < fileLines.size(); i++) {
            lexRow(fileLines.get(i));
            incrY();
        }
        CSVToken eof = CSVToken.eofTemplate.copy();
        eof.setY(getY());
        getTokenBuffer().add(eof);
        return getTokenBuffer();
    }

    public void lexRow(FileRWRow row){
        String[] buffer = row.toString().split(",");
        int length = buffer.length;

        for (int i = 0; i < length; i++) {
            getTokenBuffer().add(classify(buffer[i], getX(), getY()));
            setX(getX() + buffer[i].length());
        }
        setX(0);
    }

    public Pair<Boolean, Integer> findIn(char c, String charset){
        Boolean found;
        Integer count = 0;
        for (char test: charset.toCharArray()) {
            if (test == c){
                count++;
            }
        }

        found = count > 0;
        return new Pair<>(found, count);
    }

    public Pair<boolean[], int[]> findIn(String s, String charset){
        char[] setA = s.toCharArray();
        char[] setB = charset.toCharArray();

        boolean[] found = new boolean[setB.length];
        int[] count = new int[setB.length];

        for (int i = 0; i < setA.length; i++) {
            for (int j = 0; j < setB.length; j++) {
                if (setB[j] == setA[i]){
                    count[j]++;
                }
            }
        }

        for (int k = 0; k < count.length; k++) {
            found[k] = count[k] > 0;
        }
        return new Pair<>(found, count);
    }

    public Pair<boolean[], int[]> findIn(String s, String charset, String exclude){
        String nSet = charset;

        String[] cachedExclusion = exclude.split("");
        for (int i = 0; i < exclude.length(); i++) {
            nSet.replace(cachedExclusion[i], "");
        }

        char[] setA = s.toCharArray();
        char[] setB = nSet.toCharArray();

        boolean[] found = new boolean[setB.length];
        int[] count = new int[setB.length];

        for (int i = 0; i < setA.length; i++) {
            for (char test: setB) {
                if (test == setA[i]){
                    found[i] = true;
                    count[i]++;
                }
            }
        }

        return new Pair<>(found, count);
    }

    public int[] queryPosition(char find, String s, int expected_size){
        int[] output = new int[expected_size];
        char[] stringBuffer = s.toCharArray();
        int l = 0;
        for (int i = 0; i < stringBuffer.length; i++) {
            if (find == stringBuffer[i]){
                output[l++] = i;
            }
        }
        return output;
    }

    private boolean hasTrue(boolean[] booleans){
        boolean output = false;
        for (boolean b: booleans) {
            if (b) output = true;
        }
        return output;
    }

    private boolean[] invertBoolArray(boolean[] booleans){
        boolean[] output = new boolean[booleans.length];
        for (int i = 0; i < booleans.length; i++) {
            output[i] = !booleans[i];
        }
        return output;
    }

    private boolean quickXOR(boolean a, boolean b){
        boolean sub1 = a || b;
        boolean sub2 = !(a && b);

        return (sub1 && sub2);
    }

    private int quickIntArrSum(int[] ints){
        int output = 0;
        for (int num: ints) {
            output += num;
        }
        return output;
    }

    public CSVToken classify(String input, int col, int row){
        CSVToken output;

        Pair<boolean[], int[]> hasDigit = findIn(input, CSV_DIGITS);
        Pair<boolean[], int[]> hasLetter = findIn(input, CSV_LETTERS);
        Pair<boolean[], int[]> hasSymbol = findIn(input, CSV_SYMBOLS);

        boolean[] hasDigitInv = invertBoolArray(hasDigit.fst);
        boolean[] hasLetterInv = invertBoolArray(hasLetter.fst);
        boolean[] hasSymbolInv = invertBoolArray(hasSymbol.fst);

        boolean hasAnyDigit = hasTrue(hasDigit.fst);
        boolean hasAnyLetter = hasTrue(hasLetter.fst);
        boolean hasAnySymbol = hasTrue(hasSymbol.fst);

        int totalLetterCount = quickIntArrSum(hasLetter.snd);
        int totalDigitCount = quickIntArrSum(hasDigit.snd);
        int totalSymbolCount = quickIntArrSum(hasSymbol.snd);

        //see if and how many dots there are
        boolean symbolDotFound = hasSymbol.fst[29];
        int symbolDotCount = hasSymbol.snd[29];

        //see if and how many hyphens (minus signs) there are plus the positions of them (it matters)
        boolean symbolHyphenFound = hasSymbol.fst[10];
        int symbolHyphenCount = hasSymbol.snd[10];
        int[] symbolHyphenPos = queryPosition('-', input, symbolHyphenCount);

        //see if and how many Es there are
        boolean letterEFound = quickXOR(hasLetter.fst[9], hasLetter.fst[8]);
        int letterECount = letterEFound ? hasLetter.snd[(hasLetter.fst[9] ? 9 : 8)] : 0;
        int letterEPos = input.indexOf(CSV_LETTERS.charAt(hasLetter.fst[9] ? 9 : 8));

        int excludedLetterCount = totalLetterCount - letterECount;
        int excludedSymbolCount = totalSymbolCount - symbolHyphenCount;

        boolean negativeNumGuess = (symbolHyphenCount > 0) && symbolHyphenPos[0] == 0 ? true : false;
        boolean invalidHyphenPosition = false;

        if (symbolHyphenCount > 0){
            switch (symbolHyphenCount){
                case 1:
                    if (letterEFound){
                        invalidHyphenPosition = input.charAt(letterEPos + 1) != '-'; //see if the hyphen is after the E in sci notation
                    } else {
                        invalidHyphenPosition = symbolHyphenPos[0] != 0; //test if the num is negative (one hyphen at the start)
                    }
                    break;
                case 2:

                    if (symbolHyphenPos[0] != 0){
                        invalidHyphenPosition = true;
                        break;
                    } else {
                        invalidHyphenPosition = input.charAt(letterEPos + 1) != '-';
                    }
                    break;
            }
        } else {
            invalidHyphenPosition = false; //hyphens aren't found, so invalid placement of them can't exist
        }

        //first, try to query to see if it's a real number in proper format (INT or DOUBLE format)
        if (hasAnyDigit && (symbolHyphenCount <= 2) && !invalidHyphenPosition && excludedLetterCount == 0){
            //check if dot exists, which hints at it being a decimal or integer
            if (symbolDotFound && (symbolDotCount == 1)){
                //check if E only exists through some math tricks
                if ((excludedLetterCount == 0) && (letterEPos != input.length() - 1)){
                    (output = CSVToken.doubleTemplate.copy()).setContent(input);
                } else {
                    (output = CSVToken.stringTemplate.copy()).setContent(input);
                }
                output.setX(col);
                output.setY(row);
                return output;
            } else if (!symbolDotFound) { //likely a small integer (scientific notation exists, but I'll catch that as a double)
                (output = CSVToken.intTemplate.copy()).setContent(input);
                output.setX(col);
                output.setY(row);
                return output;
            }
        } else { //else, treat element as a STRING
            (output = CSVToken.stringTemplate.copy()).setContent(input);
            output.setX(col);
            output.setY(row);
            return output;
        }

        //if it doesn't match anything, send return as an ERROR which shouldn't really happen unless that "else" fails
        (output = CSVToken.errorTemplate.copy()).setContent(input);
        output.setX(col);
        output.setY(row);
        return output;
    }
}