package frc.lib.frc1731.hardware;

import edu.wpi.first.wpilibj.AddressableLED;

// TODO - Fix this class to actually work
public class SimpleAddressableLED {
    private AddressableLED led;

    public SimpleAddressableLED(int deviceID, int stripLength) {
        this.led = new AddressableLED(deviceID);
        led.setLength(stripLength);
    }
}