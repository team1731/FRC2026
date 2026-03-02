package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakePivotSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;

    private double rotations = 0;
    private double targetRotations = 0;

    public IntakePivotSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(Ports.kIntakePivotMotorConfig);
        // motor.getMotor().clearStickyFaults();
        motor.withPIDGains(kPivotGains);
        motor.withStatorCurrentLimit(kPivotCurrentLimit);
        motor.setSoftLimits(kPivotIntakeRotations, kPivotStowRotations);
        motor.withFeedbackConfigs(kPivotFeedbackConfigs);
        motor.withMotionMagicConfigs(kMotionMagicConfigs);
        // motor.setDynamicMotionMagicSpeeds(1.5, 2);
    }

    public boolean atTargetRotations() {
        return Utils.isWithin(rotations, targetRotations, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        rotations = motor.getRotations();

        logger.log("Current Rotations", rotations);
        logger.log("Target Rotations", targetRotations);
        logger.log("At Target Position", atTargetRotations());
    }
    
    private Command setRotations(double rotations) {
        return run(() -> {
            targetRotations = rotations;
            motor.setPosition(targetRotations);
        });
    }

    public Command deploy() {
        return this.setRotations(kPivotIntakeRotations);
    }

    public Command retract() {
        return this.setRotations(kPivotStowRotations);
    }

    public Command driveManual(double percentOutput) {
        return run(() -> motor.setPercentOutput(percentOutput));
    }

    public Command unjam() {
        // Runs the motor forward and reverse for short spurts to attempt to unjam the mechanism; Uses recursion?
        return this.driveManual(0.2)
            .withTimeout(0.5d)
            .andThen(driveManual(-0.2))
            .andThen(unjam());
    }

    public Command stop() {
        return this.driveManual(0).withTimeout(0);
    }
}