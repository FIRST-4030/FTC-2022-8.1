package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix4f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector4F;

public class DepreciatedTransform4f {

    private DepreciatedMatrix4f localRotation, localTranslation, localTransform;
    private DepreciatedMatrix4f globalRotation, globalTranslation, globalTransform;

    //default the matrices to identity matrices on construction of the object
    public DepreciatedTransform4f(){
        localRotation = new DepreciatedMatrix4f();
        localTranslation = new DepreciatedMatrix4f();
        localTransform = new DepreciatedMatrix4f();

        globalRotation = new DepreciatedMatrix4f();
        globalTranslation = new DepreciatedMatrix4f();
        globalTransform = new DepreciatedMatrix4f();
    }

    //transform vectors inputted into this

    public DepreciatedVector4F transform(DepreciatedVector4F in){
        DepreciatedVector4F out = in;
        out = localTransform.matMul(out);
        out = globalTransform.matMul(out);
        return out;
    }

    //set the local matrices

    public void replaceLocalRotation(DepreciatedMatrix4f newRotation){
        DepreciatedMatrix4f lPos = this.localTranslation;
        DepreciatedMatrix4f lRot = newRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(lPos, lRot);
    }

    public void replaceLocalTranslation(DepreciatedMatrix4f newTranslation){
        DepreciatedMatrix4f lPos = newTranslation;
        DepreciatedMatrix4f lRot = this.localRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(lPos, lRot);
    }

    public void replaceLocalTransform(DepreciatedMatrix4f newTranslation, DepreciatedMatrix4f newRotation){
        DepreciatedMatrix4f lPos = newTranslation;
        DepreciatedMatrix4f lRot = newRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(lPos, lRot);
    }


    //set the global matrices

    public void replaceGlobalRotation(DepreciatedMatrix4f newRotation){
        DepreciatedMatrix4f gPos = this.globalTranslation;
        DepreciatedMatrix4f gRot = newRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(gPos, gRot);
    }

    public void replaceGlobalTranslation(DepreciatedMatrix4f newTranslation){
        DepreciatedMatrix4f gPos = newTranslation;
        DepreciatedMatrix4f gRot = this.globalRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(gPos, gRot);
    }

    public void replaceGlobalTransform(DepreciatedMatrix4f newTranslation, DepreciatedMatrix4f newRotation){
        DepreciatedMatrix4f gPos = newTranslation;
        DepreciatedMatrix4f gRot = newRotation;

        this.localTransform = DepreciatedMatrix4f.matMul(gPos, gRot);
    }

    //get the local matrices
    public DepreciatedMatrix4f getLocalTranslation(){ return this.localTranslation; }
    public DepreciatedMatrix4f getLocalRotation() { return localRotation; }
    public DepreciatedMatrix4f getLocalTransform() { return localTransform; }

    //get the global matrices
    public DepreciatedMatrix4f getGlobalTranslation() { return globalTranslation; }
    public DepreciatedMatrix4f getGlobalRotation() { return globalRotation; }
    public DepreciatedMatrix4f getGlobalTransform() { return globalTransform; }
}
