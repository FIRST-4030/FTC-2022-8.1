package org.firstinspires.ftc.teamcode.extrautilslib.core.memsys;

/**
 * This class is primarily made to wrap any object into a generic class to take advantage of wildcards for the {@code EULMemory} class
 * @param <T>
 */
public class EULMemItem<T> {

    private T v;

    public EULMemItem(T v){
        this.v = v;
    }

    public T get(){
        return v;
    }

    public void set(T nv){
        v = nv;
    }
}
