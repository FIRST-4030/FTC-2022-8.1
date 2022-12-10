package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc.xmlrw;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;

@XmlRootElement(name = "RobotInfo")
@XmlType(propOrder = {"RobotName", "StoredRotation"})
public class RobotInfoPacket {
    private long id = 1L;
    private String robotName = "";
    private double extrinsicRotation = 0d;

    @XmlAttribute
    public RobotInfoPacket setId(long nId){
        this.id = nId;
        return this;
    }

    @XmlElement(name = "RobotName")
    public RobotInfoPacket setRobotName(String nString){
        this.robotName = nString;
        return this;
    }

    @XmlElement(name = "StoredRotation")
    public RobotInfoPacket setExtrinsicRotation(double nDouble){
        this.extrinsicRotation = nDouble;
        return this;
    }

}
