package org.firstinspires.ftc.teamcode.extrautilslib.core.filehandlers.read;

import com.sun.tools.javac.util.Pair;

import static org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULArrays.*;
import static org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULBooleans.*;
import static org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULStrings.*;

public class EULTokenizer {
    //common letters, digits, symbols
    public final static String
            DIGITS = "0123456789",
            LETTERS = "AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz",
            SYMBOLS = "~`!@#$%^&*-_+=(){}[]|\\:;\"'<,>.?/";

    public EULToken classify(String input, int col, int row){
        EULToken output;

        Pair<boolean[], int[]> hasDigit = findIn(input, DIGITS);
        Pair<boolean[], int[]> hasLetter = findIn(input, LETTERS);
        Pair<boolean[], int[]> hasSymbol = findIn(input, SYMBOLS);

        boolean[] hasDigitInv = invertBoolArray(hasDigit.fst);
        boolean[] hasLetterInv = invertBoolArray(hasLetter.fst);
        boolean[] hasSymbolInv = invertBoolArray(hasSymbol.fst);

        boolean hasAnyDigit = hasTrue(hasDigit.fst);
        boolean hasAnyLetter = hasTrue(hasLetter.fst);
        boolean hasAnySymbol = hasTrue(hasSymbol.fst);

        int totalLetterCount = intArraySum(hasLetter.snd);
        int totalDigitCount = intArraySum(hasDigit.snd);
        int totalSymbolCount = intArraySum(hasSymbol.snd);

        //see if and how many dots there are
        boolean symbolDotFound = hasSymbol.fst[29];
        int symbolDotCount = hasSymbol.snd[29];

        //see if and how many hyphens (minus signs) there are plus the positions of them (it matters)
        boolean symbolHyphenFound = hasSymbol.fst[10];
        int symbolHyphenCount = hasSymbol.snd[10];
        int[] symbolHyphenPos = queryPosition('-', input, symbolHyphenCount);

        //see if and how many Es there are
        boolean letterEFound = xor(hasLetter.fst[9], hasLetter.fst[8]);
        int letterECount = letterEFound ? hasLetter.snd[(hasLetter.fst[9] ? 9 : 8)] : 0;
        int letterEPos = input.indexOf(LETTERS.charAt(hasLetter.fst[9] ? 9 : 8));

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
                    (output = EULToken.doubleTemplate.copy()).setLexeme(input);
                } else {
                    (output = EULToken.stringTemplate.copy()).setLexeme(input);
                }
                output.setX(col);
                output.setY(row);
                return output;
            } else if (!symbolDotFound) { //likely a small integer (scientific notation exists, but I'll catch that as a double)
                (output = EULToken.intTemplate.copy()).setLexeme(input);
                output.setX(col);
                output.setY(row);
                return output;
            }
        } else { //else, treat element as a STRING
            (output = EULToken.stringTemplate.copy()).setLexeme(input);
            output.setX(col);
            output.setY(row);
            return output;
        }

        //if it doesn't match anything, send return as an ERROR which shouldn't really happen unless that "else" fails
        (output = EULToken.errorTemplate.copy()).setLexeme(input);
        output.setX(col);
        output.setY(row);
        return output;
    }
}
