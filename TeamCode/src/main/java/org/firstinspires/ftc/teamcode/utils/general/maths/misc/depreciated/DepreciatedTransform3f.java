package org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.matrices.DepreciatedMatrix3f;
import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;

public class DepreciatedTransform3f {

    private DepreciatedMatrix3f localRotation, localTranslation, localTransform;
    private DepreciatedMatrix3f globalRotation, globalTranslation, globalTransform;

    //default the matrices to identity matrices on construction of the object
    public DepreciatedTransform3f(){
        localRotation = new DepreciatedMatrix3f();
        localTranslation = new DepreciatedMatrix3f();
        localTransform = new DepreciatedMatrix3f();

        globalRotation = new DepreciatedMatrix3f();
        globalTranslation = new DepreciatedMatrix3f();
        globalTransform = new DepreciatedMatrix3f();
    }

    //transform vectors inputted into this

    public DepreciatedVector3F transform(DepreciatedVector3F in){
        DepreciatedVector3F out = in;
        out = localTransform.matMul(out);
        out = globalTransform.matMul(out);
        return out;
    }

    //set the local matrices

    public void replaceLocalRotation(DepreciatedMatrix3f newRotation){
        DepreciatedMatrix3f lPos = this.localTranslation;
        DepreciatedMatrix3f lRot = newRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(lPos, lRot);
    }

    public void replaceLocalTranslation(DepreciatedMatrix3f newTranslation){
        DepreciatedMatrix3f lPos = newTranslation;
        DepreciatedMatrix3f lRot = this.localRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(lPos, lRot);
    }

    public void replaceLocalTransform(DepreciatedMatrix3f newTranslation, DepreciatedMatrix3f newRotation){
        DepreciatedMatrix3f lPos = newTranslation;
        DepreciatedMatrix3f lRot = newRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(lPos, lRot);
    }


    //set the global matrices

    public void replaceGlobalRotation(DepreciatedMatrix3f newRotation){
        DepreciatedMatrix3f gPos = this.globalTranslation;
        DepreciatedMatrix3f gRot = newRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(gPos, gRot);
    }

    public void replaceGlobalTranslation(DepreciatedMatrix3f newTranslation){
        DepreciatedMatrix3f gPos = newTranslation;
        DepreciatedMatrix3f gRot = this.globalRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(gPos, gRot);
    }

    public void replaceGlobalTransform(DepreciatedMatrix3f newTranslation, DepreciatedMatrix3f newRotation){
        DepreciatedMatrix3f gPos = newTranslation;
        DepreciatedMatrix3f gRot = newRotation;

        this.localTransform = DepreciatedMatrix3f.matMul(gPos, gRot);
    }

    //get the local matrices
    public DepreciatedMatrix3f getLocalTranslation(){ return this.localTranslation; }
    public DepreciatedMatrix3f getLocalRotation() { return localRotation; }
    public DepreciatedMatrix3f getLocalTransform() { return localTransform; }

    //get the global matrices
    public DepreciatedMatrix3f getGlobalTranslation() { return globalTranslation; }
    public DepreciatedMatrix3f getGlobalRotation() { return globalRotation; }
    public DepreciatedMatrix3f getGlobalTransform() { return globalTransform; }
}
