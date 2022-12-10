package org.firstinspires.ftc.teamcode.utils.fileRW.xmlrw;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

public class XMLRW {

    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/ROBOT_XML_DATA/";
    private static boolean initialized = false;

    private XMLRW(){}

    public static void init(){
        if (!initialized) {
            File directory = new File(defaultDir);
            if (!directory.exists()) {
                directory.mkdir();
            }
            initialized = true;
        }
    }

    public static void marshal(RobotInfoPacket packet, String packetName) throws JAXBException {
        JAXBContext ctx = JAXBContext.newInstance(packet.getClass());
        Marshaller marshaller = ctx.createMarshaller();
        marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
        marshaller.marshal(packet, new File(defaultDir + packetName));
    }

    public static <T extends RobotInfoPacket> T unmarshal(T templateClass, String packetName) throws FileNotFoundException, JAXBException {
        JAXBContext context = JAXBContext.newInstance(templateClass.getClass());
        return (T) context.createUnmarshaller().unmarshal(new BufferedReader(new FileReader(defaultDir + packetName)));
    }
}
