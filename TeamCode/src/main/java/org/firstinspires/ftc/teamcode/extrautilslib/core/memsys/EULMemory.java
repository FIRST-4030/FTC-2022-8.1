package org.firstinspires.ftc.teamcode.extrautilslib.core.memsys;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

/**
 * Although it is called EULMemory, it is more akin to an "ID to Object" system that closely resembles a pointer system without the fancy stuff provided by the language
 */
public class EULMemory {

    private static EULMemory master = null;
    private static boolean initialized = false;

    private HashMap<Integer, EULMemItem<?>> memoryMap;
    private ArrayList<Integer> takenHexes;


    private EULMemory(){
        this.memoryMap = new HashMap<>();
        this.takenHexes = new ArrayList<>();
    }

    /**
     * The primary use of this method is for instantiating the master EULMemory where you can statically use the provided {@code alloc(...)}, {@code delete(...)}, {@code getItem(...)}, and {@code setItem(...)}
     */
    public static void init(){
        if(master == null){
            master = new EULMemory();
            initialized = true;
        }
    }

    /**
     * The primary use of this method is for grabbing the EULMemory that has been statically declared in this class
     * @return
     */
    public static EULMemory getMaster(){
        if (initialized) return EULMemory.master;
        else throw new RuntimeException("Master Memory is not initialized!");
    }

    /**
     * This method provides one of the essential functions to an ID system: allocating an ID for an object and returning said ID
     * <br>The approach for storing it doesn't necessarily store the raw object, but it makes it into a EULMemItem so it can take advantage of wildcard generic parameters
     * @param value
     * @return ID
     * @param <T>
     */
    public <T> int alloc(T value) {
        int hexCode = 0x00000000;

        if (this.takenHexes.size() == 0 || this.takenHexes.get(0) != 0) {
            this.memoryMap.put(hexCode, new EULMemItem(value));
            this.takenHexes.add(hexCode);
            Collections.sort(takenHexes);
            return hexCode;
        }



        int takenLen = this.takenHexes.size();
        int currentHex, nextHex, diff;

        for (int i = 0; i < takenLen; i++) {
            currentHex = this.takenHexes.get(i);
            nextHex = (i + 1 != takenLen) ? this.takenHexes.get(i + 1) : -3;

            switch (nextHex){
                case -3: //-3 is an error code; look for it; case if all hexes are taken
                    currentHex += 0x00000001;

                    this.takenHexes.add(currentHex);
                    this.memoryMap.put(currentHex, new EULMemItem<>(value));
                    Collections.sort(takenHexes);
                    return currentHex;
                default:
                    diff = nextHex - currentHex;
                    if (diff > 1){ //if the difference is non-one, there's space for allocation in between
                        currentHex += 0x00000001;

                        this.takenHexes.add(i + 1, currentHex);
                        this.memoryMap.put(currentHex, new EULMemItem<>(value));
                        Collections.sort(takenHexes);
                        return currentHex;
                    }
            }
        }

        return hexCode;
    }

    /**
     * This method deletes the EULMemItem at the ID and removes the ID, essentially freeing it
     * @param hex
     */
    public void delete(int hex){
        if (this.memoryMap.containsKey(hex) && this.memoryMap.get(hex).get() != null){
            this.memoryMap.remove(hex);
            this.takenHexes.remove((Integer) hex); //anti auto-casting hackery
        }
    }

    /**
     * Removes all objects and stored IDs indiscriminately
     */
    public void clearAll(){
        this.memoryMap.clear();
        this.takenHexes.clear();
    }

    /**
     * The method is self-explanatory, but for clarification, this will return the EULMemItem at the given ID, which you can use the {@code get()} method to retrieve the object without casting it into a specific class
     * @param hex
     * @return
     */
    public EULMemItem<?> getItem(int hex){
        return this.memoryMap.get(hex);
    }

    /**
     * This method is also self-explanatory, but this generic static method just replaces the object at the given ID
     * @param hex
     * @param value
     * @param <T>
     */
    public <T> void setItem(int hex, T value){
        if (this.memoryMap.containsKey(hex)) this.memoryMap.put(hex, new EULMemItem<>(value));
    }

    /**
     * The method returns what IDs are taken into an ArrayList where all normal ArrayList operations can happen, such as querying the first element or its length
     * @return
     */
    public ArrayList<Integer> getTakenHexes(){
        return this.takenHexes;
    }
}
