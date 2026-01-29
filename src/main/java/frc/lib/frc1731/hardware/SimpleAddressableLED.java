package frc.lib.frc1731.hardware;

import edu.wpi.first.wpilibj.AddressableLED;

// TODO - Fix this class to actually work
public class SimpleAddressableLED {
    private AddressableLED led;

    public SimpleAddressableLED(int port) {
        this.led = new AddressableLED(port);
    }
}