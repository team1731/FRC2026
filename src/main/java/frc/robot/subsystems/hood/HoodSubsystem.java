package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.robot.Robot;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;

    private double targetPosition = 0.0;

    private PIDGains positionGains = new PIDGains()
        .setP(1.0d);
    
    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(new PortConfig(0));
        motor.withPIDGains(positionGains);
        positionGains.logOnAdvantageScope();
        Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
            motor.withPIDGains(positionGains);
        }));
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", motor.getRotations());
        logger.log("Current Degrees", motor.getRotations());
        logger.log("Target Position", targetPosition);
        logger.log("Target Degrees", targetPosition);
    }

    public Command setPositionCommand(double position) {
        return run(() -> {
            targetPosition = position;
            motor.setPosition(position);
        });
    }

    public Command setHoodDegreesCommand(double degrees) {
        return run(() -> {
            double rotations = (degrees - kStartDegrees) / 360d / kGearRatio;
            targetPosition = rotations;
            motor.setPosition(rotations);
        });
    }
}