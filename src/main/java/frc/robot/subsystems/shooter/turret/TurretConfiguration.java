package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.frc1731.hardware.motor.PortConfig;

public record TurretConfiguration(String name, PortConfig motorConfigs, int cancoderID, 
                                    FeedbackConfigs feedbackConfigs, CANcoderConfiguration cancoderConfigs,
                                        double maxDegrees, double minDegrees, Transform3d robotToTurret) {}