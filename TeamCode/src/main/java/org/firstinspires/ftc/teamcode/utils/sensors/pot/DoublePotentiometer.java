package org.firstinspires.ftc.teamcode.utils.sensors.pot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.fileRW.ConfigFileUtil;

import java.util.ArrayList;

public class DoublePotentiometer implements Potentiometer {

    //hardware
    private BasicPotentiometer pot1a, pot1b, pot2a, pot2b;
    private BasicPotentiometer pot1, pot2;

    private final double scaleRad = Math.PI / 180;

    //potentiometer tuning for linearization - for later
    public final double OFFSET;
    private static double CLIP_TOP = 30;
    private static double CLIP_BOTTOM = 30;

    double thr1 = 0.332;
    double thr2 = 0.282;

    double tolerance = 0.2;

    //initalize hardware
    public DoublePotentiometer(HardwareMap map, Telemetry telemetry, String n1, String n2, double offset, double[] poly1, double[] poly2, double[] poly3, double[] poly4, double[] cracker1, double[] cracker2, double[] cracker3, double[] cracker4){
        if(n1 == null || n2 == null || n1.isEmpty() || n2.isEmpty()){
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": invalid name");
        }
        ArrayList normalized = new ArrayList<Double>();
        normalized.add(0);
        normalized.add(1);

        pot1a = new BasicPotentiometer(map, telemetry, n1, poly1, cracker1);
        pot2a = new BasicPotentiometer(map, telemetry, n2, poly2, cracker2);
        pot1b = new BasicPotentiometer(map, telemetry, n1, poly3, cracker3);
        pot2b = new BasicPotentiometer(map, telemetry, n2, poly4, cracker4);
        pot1 = new BasicPotentiometer(map, telemetry, n1, new double[] {0,1}, new double[] {0,1});
        pot2 = new BasicPotentiometer(map, telemetry, n2, new double[] {0,1}, new double[] {0,1});
        this.OFFSET = offset;
    }

    public static DoublePotentiometer FromData(HardwareMap map, Telemetry telemetry, String n1, String n2, double offset) {
        double[] poly1 = {0.0, 0.0, 0.0, 0.006, 0.025, 0.031, 0.045, 0.064, 0.075, 0.089, 0.1, 0.107, 0.117, 0.123, 0.137, 0.145, 0.157, 0.161, 0.168, 0.17400000000000002, 0.18, 0.19, 0.196, 0.203, 0.211, 0.216, 0.219, 0.228, 0.23600000000000002, 0.249, 0.258, 0.262, 0.275, 0.28400000000000003, 0.293, 0.307, 0.317, 0.332, 0.34900000000000003, 0.36, 0.376, 0.387, 0.402, 0.421, 0.434, 0.466, 0.492, 0.52, 0.555, 0.608, 0.66, 0.708, 0.755, 0.8190000000000001, 0.884, 0.996, 1.051, 1.245, 1.407, 1.614, 1.929, 2.562};
        double[] poly2 = {0.34700000000000003, 0.341, 0.336, 0.328, 0.323, 0.316, 0.305, 0.302, 0.293, 0.29, 0.28300000000000003, 0.273, 0.267, 0.263, 0.25, 0.246, 0.23900000000000002, 0.234, 0.228, 0.221, 0.21, 0.20500000000000002, 0.196, 0.191, 0.184, 0.17500000000000002, 0.166, 0.159, 0.147, 0.14100000000000001, 0.129, 0.112, 0.10200000000000001, 0.09, 2.46, 3.222, 3.188, 2.47, 1.963, 1.6280000000000001, 1.3900000000000001, 1.258, 1.1520000000000001, 1.0030000000000001, 0.932, 0.854, 0.798, 0.739, 0.671, 0.647, 0.608, 0.5640000000000001, 0.543, 0.512, 0.489, 0.47000000000000003, 0.442, 0.42, 0.40700000000000003, 0.383, 0.369, 0.356};
        double[] poly3 = {3.0540000000000003, 2.952, 2.868, 2.3040000000000003, 1.741, 1.495, 1.229, 1.114, 0.982, 0.862, 0.804, 0.711, 0.684, 0.634, 0.592, 0.55, 0.521, 0.501, 0.468, 0.443, 0.423, 0.395, 0.373, 0.363, 0.35100000000000003, 0.34, 0.323, 0.31, 0.296, 0.29, 0.275, 0.267, 0.256, 0.246, 0.23800000000000002, 0.229, 0.219, 0.213, 0.20600000000000002, 0.195, 0.184, 0.177, 0.164, 0.155, 0.14400000000000002, 0.133, 0.122, 0.107, 0.098, 0.073, 0.053, 0.036000000000000004, 0.001, 0.0};
        double[] poly4 = {0.058, 0.03, 0.003, 0.0, 0.001, 0.019, 0.047, 0.076, 0.095, 0.108, 0.12, 0.135, 0.147, 0.156, 0.168, 0.179, 0.184, 0.201, 0.20700000000000002, 0.217, 0.224, 0.23, 0.23900000000000002, 0.251, 0.256, 0.272, 0.28, 0.296, 0.305, 0.316, 0.329, 0.35100000000000003, 0.355, 0.376, 0.399, 0.422, 0.449, 0.47300000000000003, 0.499, 0.528, 0.548, 0.582, 0.627, 0.665, 0.709, 0.78, 0.854, 0.934, 1.064, 1.179, 1.327, 1.489, 1.672, 2.115};
        double[] cracker1 = {3.3103447187244575, 5.7931032577678, 7.448275617130029, 9.103447976492257, 10.758620335854486, 12.413792695216715, 14.4827581444195, 16.758620138542565, 18.413792497904794, 20.68965449202786, 23.1724130310712, 25.241378480273987, 27.72413701931733, 30.206895558360674, 32.89655064232429, 35.37930918136764, 37.862067720410984, 40.551722804374606, 42.82758479849767, 45.31034333754101, 48.620688056265465, 51.31034314022909, 54.41379131403327, 57.10344639799689, 59.79310148196051, 62.89654965576469, 65.99999782956887, 68.68965291353248, 71.5862045424164, 74.27585962638001, 77.79310089002475, 80.68965251890864, 83.99999723763311, 86.89654886651701, 89.58620395048062, 93.31034175904564, 96.41378993284982, 99.31034156173372, 102.82758282537846, 106.13792754410291, 108.62068608314625, 111.72413425695044, 114.62068588583433, 117.9310306045588, 120.8275822334427, 124.5517200420077, 128.06896130565244, 131.58620256929717, 135.3103403778622, 139.2413747313475, 142.75861599499223, 146.06896071371668, 149.37930543244113, 152.89654669608586, 155.79309832496978, 158.89654649877394, 161.79309812765786, 164.89654630146202, 168.41378756510676, 172.34482191859206, 175.24137354747597, 178.7586148111207};
        double[] cracker2 = {4.34482744332585, 6.413792892528636, 8.275861796811144, 9.517241066332815, 11.172413425695044, 13.24137887489783, 15.517240869020894, 17.3793097733034, 19.24137867758591, 21.724137216629252, 23.999999210752314, 26.068964659955103, 28.551723198998445, 31.241378282962067, 33.72413682200541, 36.41379190596903, 38.89655044501237, 41.37930898405572, 43.86206752309906, 46.3448260621424, 49.86206732578714, 52.34482586483048, 55.44827403863466, 57.931032577678, 60.8275842065619, 63.931032380366084, 67.03448055417026, 69.5172390932136, 72.82758381193806, 75.51723889590168, 79.03448015954642, 81.93103178843032, 85.24137650715478, 87.72413504619811, 271.4482669354055, 275.1724047439705, 278.27585291777467, 281.3793010915789, 284.48274926538306, 287.58619743918723, 291.7241283375928, 295.44826614615783, 298.96550740980257, 302.275852128527, 305.3793003023312, 308.4827484761354, 311.7930931948598, 315.1034379135843, 318.620679177229, 322.344816985794, 325.86205824943875, 329.1724029681632, 332.2758511419674, 335.5861958606919, 339.1034371243366, 342.62067838798134, 346.1379196516261, 349.44826437035056, 352.5517125441547, 355.86205726287915, 358.3448158019225, 359.79309161636445};
        double[] cracker3 = {182.06895952984516, 185.99999388333046, 188.89654551221435, 192.41378677585908, 195.72413149458353, 199.24137275822827, 202.55171747695275, 206.06895874059748, 209.37930345932193, 212.89654472296667, 216.20688944169112, 219.72413070533585, 223.0344754240603, 225.93102705294422, 229.03447522674838, 232.13792340055258, 235.65516466419732, 238.96550938292177, 242.89654373640707, 246.82757808989234, 250.75861244337764, 254.0689571621021, 256.965508790986, 260.2758535097105, 263.17240513859434, 266.4827498573188, 269.7930945760433, 273.93102547444886, 277.2413701931733, 279.7241287322166, 283.0344734509411, 286.3448181696656, 290.0689559782306, 293.5861972418753, 297.3103350504403, 300.6206797691648, 303.93102448788926, 307.0344726616934, 310.34481738041785, 313.65516209914233, 316.7586102729465, 320.6896446264318, 324.6206789799171, 327.72412715372127, 330.82757532752544, 334.3448165911702, 337.4482647649744, 340.96550602861913, 344.6896438371841, 347.7930920109883, 351.10343672971277, 354.20688490351694, 357.3103330773211, 359.1724019816036};
        double[] cracker4 = {91.24137630984285, 94.34482448364703, 97.6551692023715, 100.75861737617566, 104.27585863982041, 107.17241026870431, 109.65516880774766, 113.17241007139239, 116.27585824519656, 118.96551332916019, 122.48275459280492, 126.20689240136994, 129.5172371200944, 133.03447838373913, 136.96551273722443, 140.8965470907097, 144.20689180943418, 147.31033998323835, 150.82758124688308, 154.34482251052782, 157.03447759449145, 160.13792576829562, 163.2413739420998, 166.34482211590398, 170.27585646938928, 173.79309773303402, 176.6896493619179, 180.41378717048292, 183.93102843412765, 187.03447660793185, 190.3448213266563, 193.86206259030104, 196.9655107641052, 200.8965451175905, 203.99999329139467, 207.7241310999597, 211.03447581868414, 214.34482053740862, 217.86206180105336, 221.1724065197778, 224.27585469358198, 227.58619941230646, 230.48275104119034, 233.7930957599148, 237.10344047863927, 240.82757828720426, 244.96550918560985, 248.68964699417486, 252.41378480273988, 255.51723297654405, 258.6206811503482, 261.51723277923213, 264.41378440811604, 267.72412912684047};

        return new DoublePotentiometer(map, telemetry, n1, n2, offset, poly1, poly2, poly3, poly4, cracker1, cracker2, cracker3, cracker4);


    }



    // see overridden methods
    // some (marked with numbers) allow acces to the individual potentiometers inside the dual unit

    @Override
    public double getMV() {
        return 0;
    }

    public double getMV1(){
        return pot1.getMV();
    }

    public double getMV2(){
        return pot2.getMV();
    }

    @Override
    public double getAngleD() {
        double angle1 = pot1a.getAngleD();
        double angle2 = pot2a.getAngleD();
        double angle3 = pot1b.getAngleD();
        double angle4 = pot2b.getAngleD();

        double mv1 = pot1a.getMV();
        double mv2 = pot2b.getMV();

        double angleOut = 0;
        double n = 0;

        if(mv2 < thr1-tolerance){
            n++;
            angleOut+= angle1;
        }
        if(mv2 > thr1+tolerance){
            n++;
            angleOut+= angle3;
        }
        if(mv1 < thr2-tolerance){
            n++;
            angleOut+= angle2;
        }
        if(mv1 > thr1+tolerance){
            n++;
            angleOut+= angle4;
        }

        if(n==0)return 0;
        return angleOut/n;
    }

    @Override
    public double getAngleR() {
        return Math.toRadians(this.getAngleD());//*scaleRad;
    }

    public boolean isZeroOne(){
        return pot1.isZero();
    }

    public boolean isZeroTwo(){
        return pot2.isZero();
    }

    @Override
    public boolean isZero(){
        return getAngleD() == 0;
    }

    @Override
    public boolean isAvailable() {
        return false;
    }

    public double getAngleD1a(){
        return pot1a.getAngleD();
    }
    public double getAngleD1b(){
        return pot1b.getAngleD();
    }
    public double getAngleD2a(){
        return pot2a.getAngleD();
    }
    public double getAngleD2b(){
        return pot2b.getAngleD();
    }

    public static ArrayList<Double> toArrayList(double[] meow){
        ArrayList<Double> caw = new ArrayList<Double>();
        for(double d : meow) caw.add(d);
        return caw;
    }
}
