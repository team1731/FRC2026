package frc.lib.frc1731.hardware.motor;

import com.ctre.phoenix6.CANBus;

import frc.robot.RobotConstants;

/**
 * Basic CAN device address consisting of a bus and device ID.
 */
public class PortConfig {
    /** CAN bus this device is wired to. */
    public final CANBus kBus;

    /** CAN ID or device port for this hardware device. */
    public final int kPort;

    /**
     * Creates a port config on a specific CAN bus.
     *
     * @param bus CAN bus for the device
     * @param port CAN ID or device port
     */
    public PortConfig(CANBus bus, int port) {
        this.kBus = bus;
        this.kPort = port;
    }

    /**
     * Creates a port config on the main robot CAN bus.
     *
     * @param port CAN ID or device port
     */
    public PortConfig(int port) {
        this(RobotConstants.kMainCANBus, port);
        if (port > 62 || port < 0) {
            throw new IllegalArgumentException("Port cannot be greater than 62 or less than 0");
        }
    }
}
