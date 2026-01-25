package frc.robot.subsystems.turret;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private double targetDegrees = 0.0;

    public TurretSubsystem(boolean leftTurret, boolean enabled) {
        super(enabled);
        if (!enabled) return;

        this.withConverter(new AngularSubsystemConverter(kGearRatio));
        motor = new MotorIOTalonFX(kLeftPortConfigs);
        motor.withPIDGains(kPositionGains);
    }

    private double toMotorRotations(double turretDegrees) {
        return turretDegrees / (360d * kGearRatio);
    }

    private double toTurretDegrees(double motorRotations) {
        return motorRotations * 360d * kGearRatio;
    }

    public boolean atTargetPosition() {
        return Utils.isWithin(motor.getRotations(), targetDegrees, kEpsilon);
    }

    public Command setDegreesCommand(double degrees) {
        return this.run(() -> motor.setPosition(toMotorRotations(degrees)));
    }

    public Command setRotationsCommand(double rotations) {
        return this.run(() -> motor.setPosition(rotations));
    }

    public Command setPercentOutputCommand(double percent) {
        return this.run(() -> motor.setPercentOutput(percent));
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", motor.getRotations());
        logger.log("Current Degrees", toTurretDegrees(motor.getRotations()));
        logger.log("Target Rotations", toMotorRotations(targetDegrees));
        logger.log("Target Degrees", targetDegrees);
        logger.log("At Target Position", atTargetPosition());
    }
}