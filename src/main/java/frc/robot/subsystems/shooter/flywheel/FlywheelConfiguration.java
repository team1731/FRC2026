package frc.robot.subsystems.shooter.flywheel;

import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.subsystems.SubsystemConfiguration;

public record FlywheelConfiguration(String name, PortConfig portConfig) implements SubsystemConfiguration {
    
}