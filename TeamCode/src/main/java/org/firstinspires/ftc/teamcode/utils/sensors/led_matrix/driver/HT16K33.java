package org.firstinspires.ftc.teamcode.utils.sensors.led_matrix.driver;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "8x8 LED Matrix", description = "8x8 LED Matrix with one color", xmlTag = "BI_8x8")
public class HT16K33 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // =========
    // Our Stuff
    // =========

    /*
     * Registers
     * First four bits are the actual register, second four bits are data
     *
     * Documentation for these is on page 24:
     * https://www.holtek.com/documents/10179/116711/HT16K33v120.pdf
     */
    private static final byte DISPLAY_DATA = (byte) 0b00000000;
    private static final byte SYSTEM_SETUP = (byte) 0b00100000;
    private static final byte KEY_DATA = (byte) 0b01000000;
    private static final byte INT_FLAG = (byte) 0b01100000;
    private static final byte DISPLAY_SETUP = (byte) 0b10000000;
    private static final byte ROW_INT_SET = (byte) 0b10100000;
    private static final byte DIMMING_SET = (byte) 0b11100000;

    // Default I2C address
    public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    // Array holding display state
    public byte[] displayBuffer = new byte[16];
    /*
     * The matrix has it's own internal RAM that stores 16 bytes.
     *  (16 bytes bc it's made to support up to 8x16 displays)
     * Each byte corresponds to a row on the display.
     * The RAM is laid out in a grid, two columns and eight rows, starting from the top left.
     *
     * Index  |   Col 1 (even)  |    Col 2 (odd)  |
     * 0, 1   | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 2, 3   | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 4, 5   | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 6, 7   | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 8, 9   | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 10, 11 | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 12, 13 | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     * 14, 15 | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 0 0 |
     *
     * 1 means that LED in that row will be on, 0 means it will be off.
     *
     * On an 8x8 display we only care about the first column of data, the second column doesn't
     *  do anything.
     * On an 8x8 bi-color display each column corresponds to one color
     *
     * Godspeed, you glorious bastard
     */

    private boolean initialized = false;

    private boolean init() {

        if (!initialized) {
            // Enable clock
            deviceClient.write8((byte) (SYSTEM_SETUP | 0b0001), 0);

            // set blinking mode
            deviceClient.write8((byte) (DISPLAY_SETUP | 0b0001), 0);

            // set brightness
            deviceClient.write8((byte) (DIMMING_SET | 0b1111), 0);

            initialized = true;
        }

        return initialized;
    }

    /**
     * Clears the display buffer
     */
    public void clear() {
        for (int i = 0; i < 16; i++) {
            displayBuffer[i] = 0b00000000;
        }
    }

    /**
     * Writes the displayBuffer to the matrix
     *
     * This needs to be called to update the display
     */
    public void write() {
        // Mirror the output so it works more intuitively
        byte[] temp = new byte[16];
        for (int i = 0; i < displayBuffer.length; i++) {
            temp[i] = (byte) (Integer.reverse(displayBuffer[i]) >> 24);
        }

        deviceClient.write(DISPLAY_DATA, temp);
    }

    /**
     * Sets the brightness of the matrix
     *
     * @param brightness Brightness from 1-16
     */
    public void setBrightness(int brightness) {
        // Cap brightness value
        if (brightness < 1) brightness = 1;
        if (brightness > 16) brightness = 16;

        // adjust down by one
        brightness--;

        // Write to the display
        deviceClient.write8((byte) (DIMMING_SET | (byte)brightness), 0);
    }

    /**
     * Sets the blinking mode of the display
     *
     * BLINKING_MODE.OFF does not turn the display off,
     * it disables blinking
     *
     * @param mode Blinking mode
     */
    public void setBlinking(BLINKING_MODE mode) {
        // Write mode to the display
        deviceClient.write8((byte) (DISPLAY_SETUP | 0b0001 | (byte)mode.getMode()), 0);
    }

    // ================
    // I2C Driver Stuff
    // ================

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "8x8 LED Matrix";
    }

    public HT16K33(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        return init();
    }
}
