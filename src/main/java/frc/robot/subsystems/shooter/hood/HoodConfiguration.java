package frc.robot.subsystems.shooter.hood;

import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.subsystems.SubsystemConfiguration;

public record HoodConfiguration(String name, PortConfig portConfig) implements SubsystemConfiguration {}