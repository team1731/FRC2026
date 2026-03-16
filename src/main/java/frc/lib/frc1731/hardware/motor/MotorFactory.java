package frc.lib.frc1731.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;

/**
 * Utility class for creating and managing motors
 * 
 * TODO - Probably needs testing or fixing but I'm too lazy now to do it
 */
public class MotorFactory {
    private static List<MotorIO> motors = new ArrayList<>();

    public static MotorIOTalonFX createDefaultTalonFX(PortConfig config) {
        int port = config.kPort;
        if (motors.get(port) == null) {
            motors.add(port, new MotorIOTalonFX(config));
        }

        return (MotorIOTalonFX) motors.get(port);
    }

    public static <M extends MotorIO> boolean isOfMotorType(int port, Class<M> cls) {
        return motors.get(port) != null && motors.get(port).getClass().equals(cls);
    }
}
