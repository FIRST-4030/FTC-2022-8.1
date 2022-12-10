package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw.RobotInfoPacket;
import org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw.XMLRW;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

public class AngleOffsetHandler {

    public double rawAngle = 0;

    public AngleOffsetHandler(){
        XMLRW.init();
    }

    public AngleOffsetHandler recordAngle(BNO055IMU imu){
        rawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        return this;
    }

    public AngleOffsetHandler toXML() throws JAXBException {
        RobotInfoPacket packet = new RobotInfoPacket();
        packet.setExtrinsicRotation(rawAngle);
        XMLRW.marshal(packet, "FieldCentricOffset.xml");
        return this;
    }

    public double fromXML() throws JAXBException, FileNotFoundException {
        return XMLRW.unmarshal(new RobotInfoPacket(), "FieldCentricOffset.xml").getExtrinsicRotation();
    }
}
