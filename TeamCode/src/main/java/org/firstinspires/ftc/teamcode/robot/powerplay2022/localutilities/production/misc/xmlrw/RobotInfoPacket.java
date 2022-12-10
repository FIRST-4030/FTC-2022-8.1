package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.xmlrw;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

@XmlRootElement(name = "RobotInfo")
@XmlType(propOrder = {"RobotName", "StoredRotation"})
public class RobotInfoPacket {

    private String robotName = "";
    private long identifier = 1l;
    private double extrinsicRotation = 0d;

    @XmlElement(name = "StoredRotation")
    public RobotInfoPacket setExtrinsicRotation(double nDouble){
        this.extrinsicRotation = nDouble;
        return this;
    }

    @XmlElement(name = "RobotName")
    public RobotInfoPacket setRobotName(String nString){
        this.robotName = nString;
        return this;
    }
}
