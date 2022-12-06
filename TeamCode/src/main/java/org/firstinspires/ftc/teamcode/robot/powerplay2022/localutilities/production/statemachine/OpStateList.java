package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.statemachine;

import java.util.ArrayList;
import java.util.Arrays;

public class OpStateList {

    public ArrayList<OpState> states; // stores a mutable list of states
    private int index; //track the current index it is on

    public OpStateList(){
        this.index = 0;
        this.states = new ArrayList<>();
    }


    public void addStates(OpState... states){
        this.states.addAll(Arrays.asList(states));
    }

    public void removeStates(int... indices){
        Arrays.sort(indices); //sorts into ascending order

        //iterate backwards since I don't want to do fancy math for forward iteration and index-removal compensation
        for (int i = indices.length - 1; i >= 0; i--) {
            states.remove(indices[i]);
        }
    }

    public OpState[] getAsArray(){
        return states.toArray(new OpState[0]);
    }

    public OpState getNextState(){
        return index + 1 < states.size() ? states.get(index + 1) : null;
    }

    public OpState getCurrentState(){
        return states.get(index);
    }

    public OpState getLastState(){
        return index - 1 >= 0 ? states.get(index - 1) : null;
    }

    public int getIndex(){
        return index;
    }

    public void setIndex(int nIndex){
        if (nIndex < states.size() && nIndex >= 0) {
            index = nIndex;
        } else {
            throw new ArrayIndexOutOfBoundsException(OpStateList.class.getSimpleName() + " refused the new index.");
        }
    }

    public void incrementIndex(){
        if (index + 1 < states.size() && index >= 0) {
            index++;
        }
    }

    public void decrementIndex(){
        if (index < states.size() && index - 1 >= 0) {
            index--;
        }
    }
}
