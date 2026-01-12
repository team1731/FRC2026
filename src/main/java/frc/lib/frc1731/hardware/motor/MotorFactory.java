package frc.lib.frc1731.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.hardware.MotorIOTalonFX;

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

    // public static OLD_MotorIOSparkMax createDefaultSparkMax(PortConfig config) {
    //     int port = config.kPort;
    //     if (motors.get(port) == null) {
    //         motors.add(port, new OLD_MotorIOSparkMax(config));
    //     }

    //     return (OLD_MotorIOSparkMax) motors.get(port);
    // }

    // public static OLD_MotorIOSparkFlex createDefaultSparkFlex(PortConfig config) {
    //     int port = config.kPort;
    //     if (motors.get(port) == null) {
    //         motors.add(port, new OLD_MotorIOSparkFlex(config));
    //     }

    //     return (OLD_MotorIOSparkFlex) motors.get(port);
    // }

    // public static OLD_MotorIOTalonFXS createDefaultTalonFXS(PortConfig config) {
    //     int port = config.kPort;
    //     if (motors.get(port) == null) {
    //         motors.add(port, new OLD_MotorIOTalonFXS(config));
    //     }
        
    //     return (OLD_MotorIOTalonFXS) motors.get(port);
    // }

    public static <M extends MotorIO> boolean isOfMotorType(int port, Class<M> cls) {
        return motors.get(port) != null && motors.get(port).getClass().equals(cls);
    }
}
