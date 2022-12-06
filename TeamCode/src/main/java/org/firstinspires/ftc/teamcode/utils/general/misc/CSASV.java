package org.firstinspires.ftc.teamcode.utils.general.misc;

import java.util.concurrent.ConcurrentHashMap;

/**
 * Child Specific Abstract Static Variable
 * @param <T>
 */
public class CSASV<T>{

    private ConcurrentHashMap<String, T> csashm;

    public CSASV(){
        csashm = new ConcurrentHashMap<>();
    }

    public void register(Object root, T variable){
        String simpleName = root.getClass().getSimpleName();
        if (csashm.containsKey(simpleName)) {
            return;
        }
        csashm.put(simpleName, variable);
    }

    public void deregister(Object root){
        String simpleName = root.getClass().getSimpleName();
        if (csashm.containsKey(simpleName)){
            csashm.remove(simpleName);
        } else {
            return;
        }
    }

    public void dispose(){
        csashm.clear();
    }

    public synchronized T get(Object root){
        String simpleName = root.getClass().getSimpleName();
        if (csashm.containsKey(simpleName)){
            return csashm.get(simpleName);
        } else {
            return null;
        }
    }

    public synchronized void set(Object root, T variable){
        String simpleName = root.getClass().getSimpleName();
        if (csashm.containsKey(simpleName)){
            csashm.replace(simpleName, variable);
        } else {
            return;
        }
    }


}
