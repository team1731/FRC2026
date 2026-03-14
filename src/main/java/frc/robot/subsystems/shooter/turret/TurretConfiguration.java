package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.subsystems.SubsystemConfiguration;

public record TurretConfiguration(String name, PortConfig motorConfigs, int cancoderID, 
                                    FeedbackConfigs feedbackConfigs, CANcoderConfiguration cancoderConfigs,
                                        double maxDegrees, double minDegrees, Translation3d robotToTurret) implements SubsystemConfiguration {}