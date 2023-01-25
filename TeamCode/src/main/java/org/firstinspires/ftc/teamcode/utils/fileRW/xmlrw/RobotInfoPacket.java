package org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw;

public class RobotInfoPacket {
    private long id = 1L;
    private String robotName = "";
    private double extrinsicRotation = 0d;

    public RobotInfoPacket setId(long nId){
        this.id = nId;
        return this;
    }

    public long getId(){
        return this.id;
    }

    public RobotInfoPacket setRobotName(String nString){
        this.robotName = nString;
        return this;
    }

    public String getRobotName(){
        return this.robotName;
    }

    public RobotInfoPacket setExtrinsicRotation(double nDouble){
        this.extrinsicRotation = nDouble;
        return this;
    }

    public double getExtrinsicRotation(){
        return this.extrinsicRotation;
    }
}
