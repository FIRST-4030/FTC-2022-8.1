package org.firstinspires.ftc.teamcode.robot.powerplay2022.localutilities.production.misc;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorView {
    public NormalizedRGBA colorInput;
    public double distance;

    public enum CMYcolors{
        RED,
        GREEN,
        BLUE,
        NOT_DETECTED
    }

    public ColorView(NormalizedRGBA CI, double DIST){
        colorInput = CI;
        distance = DIST;
        double total = Math.sqrt(colorInput.red*colorInput.red + colorInput.green*colorInput.green + colorInput.blue*colorInput.blue);
        colorInput.red /= total;
        colorInput.green /= total;
        colorInput.blue /= total;
    }

    public CMYcolors getColor(){
        CMYcolors output = CMYcolors.NOT_DETECTED;
        if(distance < 1000000 && distance > 0) {
            if (colorInput.red > colorInput.green && colorInput.blue < colorInput.red) {
                output = CMYcolors.RED;
            }else if (colorInput.green > colorInput.blue) {
                output = CMYcolors.GREEN;
            } else {
                output = CMYcolors.BLUE;
            }
        }
        return output;
    }

    public CMYcolors getColorBetter(float range) {
        CMYcolors output = CMYcolors.NOT_DETECTED;
        float hue = convertRGBToHSV(colorInput)[0];
        float dY= Math.abs(60-hue), dC = Math.abs(180-hue), dM = Math.abs(300-hue);
        if(dY < dC && dY < dM && dY < range){
            output = CMYcolors.BLUE;
        }
        if(dC < dY && dC < dM && dC < range){
            output = CMYcolors.BLUE;
        }
        if(dM < dC && dM < dY && dM < range){
            output = CMYcolors.BLUE;
        }
        return output;
    }

    public void update(NormalizedRGBA CI, double DIST){
        colorInput = CI;
        distance = DIST;
    }

    //code adapted from https://developer.tizen.org/community/code-snippet/web-code-snippet/rgba-hsv-conversion

    /**
     *
     * @param rgba
     * @return
     */
    public float[] convertRGBToHSV(NormalizedRGBA rgba){
        //use floating space values since  ^  returns floating points instead of doubles; save some memory
        float r = rgba.red;
        float g = rgba.green;
        float b = rgba.blue;
        float a = rgba.alpha; //this formula ignores the Alpha component, but I'll append it

        float max = Math.max(r, Math.max(g, b)); //find the highest component
        float min = Math.min(r, Math.min(g, b)); //find the lowest component
        float delta = max - min; //find space between min and max

        float h = 0, s, v = max; //HUE, SATURATION, VALUE
        s = max == 0 ? 0 : delta / max;

        if (max != min){
            if (max == r){
                h = (g - b) / delta + (g < b ? 6 : 0);
            } else if (max == g){
                h = (b - r) / delta + 2;
            } else {
                h = (r - g) / delta + 4;
            }
            h /= 6;
        }

        return new float[]{h * 360, s, v, a};
    }
}
