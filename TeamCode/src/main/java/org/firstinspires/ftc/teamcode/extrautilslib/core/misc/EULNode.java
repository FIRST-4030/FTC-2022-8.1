package org.firstinspires.ftc.teamcode.extrautilslib.core.misc;

import java.util.ArrayList;

public class EULNode<T> {

    private EULNode parent;
    private ArrayList<EULNode> children;
    private T content;
    private String name;


    public EULNode(String name, T content, EULNode parent){
        this.name = name;
        this.content = content;
        this.parent = parent;
    }



    public String getName(){
        return name;
    }

    public EULNode setName(String nName){
        name = nName;
        return this;
    }



    public T getContent(){
        return content;
    }

    public EULNode setContent(T nContent){
        content = nContent;
        return this;
    }



    public boolean hasParent(){
        return parent != null;
    }

    public EULNode getParent(){
        return parent;
    }

    public EULNode setParent(EULNode nParent){
        parent = nParent;
        return this;
    }



    public EULNode addChildren(EULNode... nodes){
        for (EULNode node: nodes) {
            node.parent = this;
            this.children.add(node);
        }
        return this;
    }

    public EULNode removeChildren(EULNode... nodes){
        for (EULNode node: nodes) {
            node.parent = null;
            this.children.remove(node);
        }
        return this;
    }

    public EULNode[] getChildren(){
        return children.toArray(new EULNode[0]);
    }

    public EULNode setChildren(EULNode... nodes){
        children.clear();
        for (EULNode node: nodes) {
            node.parent = this;
            children.add(node);
        }
        return this;
    }
}
