package org.firstinspires.ftc.teamcode.utils.pathfinder.utilities;

import org.firstinspires.ftc.teamcode.utils.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;

public abstract class PathfinderJoystickControlModule {

    public InputHandler gamePad;

    public double lateralControl, advancementControl, rotationalControl;
    public boolean fieldCentric;
    public double currentAngle;
    public boolean negateControls;

    public abstract void update();

    public Vector4d getAsVector(){
        return new Vector4d(lateralControl * (negateControls ? -1 : 1), advancementControl * (negateControls ? -1 : 1), rotationalControl * (negateControls ? -1 : 1), 1); //1 is a default value to be able to add a passive bias
    }
}
