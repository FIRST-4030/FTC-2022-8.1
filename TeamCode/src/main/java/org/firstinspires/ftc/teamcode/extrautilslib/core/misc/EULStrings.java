package org.firstinspires.ftc.teamcode.extrautilslib.core.misc;

import com.sun.tools.javac.util.Pair;

public class EULStrings {

    public static String crawlToChar(String input, int startIdx, char search){
        String output = "";
        char current = ' ';
        int idx = 0;

        while (current != search && input.indexOf(search) != -1) {
            current = input.charAt(startIdx + idx);
            output += current;
            idx++;
        }

        return output;
    }

    public static Pair<Boolean, Integer> findIn(char c, String charset){
        boolean found;
        int count = 0;
        for (char test: charset.toCharArray()) {
            if (test == c){
                count++;
            }
        }

        found = count > 0;
        return new Pair<>(found, count);
    }

    public static Pair<boolean[], int[]> findIn(String s, String charset){
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

    public static Pair<boolean[], int[]> findIn(String s, String charset, String exclude){
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

    public static int[] queryPosition(char find, String s, int expected_size){
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
}
