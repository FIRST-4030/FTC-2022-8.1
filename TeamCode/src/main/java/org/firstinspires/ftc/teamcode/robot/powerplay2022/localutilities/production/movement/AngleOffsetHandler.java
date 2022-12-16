package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.utils.fileRW.ConfigFileUtil;
import org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw.RobotInfoPacket;
import org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw.XMLRW;

import java.io.FileNotFoundException;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

public class AngleOffsetHandler {

    public double rawAngle = 0;

    public AngleOffsetHandler(){
        XMLRW.init();
        ConfigFileUtil.init();
    }

    public AngleOffsetHandler recordAngle(BNO055IMU imu){
        rawAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        return this;
    }

    public AngleOffsetHandler toXML() throws JAXBException {
        /*
        RobotInfoPacket packet = new RobotInfoPacket();
        packet.setExtrinsicRotation(rawAngle);
        XMLRW.marshal(packet, "FieldCentricOffset.xml");

         */
        ArrayList<Double> d = new ArrayList<>();
        d.add(rawAngle);
        ConfigFileUtil.writeToConfig("ROBOT_ANGLE", d,  1);
        return this;
    }

    public double fromXML() throws JAXBException, FileNotFoundException {
        Object[][] d = new Object[1][1];
        ConfigFileUtil.readConfig("ROBOT_ANGLE", d, ConfigFileUtil.ConfigDataType.DOUBLE);
        return (double) d[0][0];
    }
}
