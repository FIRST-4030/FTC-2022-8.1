package org.firstinspires.ftc.teamcode.utils.sensors.switches;

public class SwitchConfig {
    public final String name;
    public final SWITCH_TYPES type;

    public SwitchConfig(SWITCH_TYPES type, String name) {
        this.name = name;
        this.type = type;
    }
}
