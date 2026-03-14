package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

public class IntakePivotSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private double targetPosition = 0;

    public IntakePivotSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    public void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIntakePivotConfig);
        motor.getMotor().clearStickyFaults();
        motor.withPIDGains(kPivotGains);
        motor.withStatorCurrentLimit(kPivotCurrentLimit);
        motor.setSoftLimits(kPivotIntakeRotations, kPivotStowRotations);
        motor.withFeedbackConfigs(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(Ports.kPivotCANcoderId)
            .withRemoteCANcoder(new CoreCANcoder(Ports.kPivotCANcoderId))
            .withRotorToSensorRatio(kPivotGearRatio)
        );

        motor.withMotionProfile(3, 2);
        motor.setDynamicMotionMagicSpeeds(3, 2);
    }

    public boolean atTargetPosition() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations(), targetPosition, 0.01);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", motor.getRotations());
        logger.log("Target Rotations", targetPosition);
        logger.log("At Target Position", atTargetPosition());
    }
    
    private Command setPosition(double position) {
        return run(() -> {
            targetPosition = position;
            motor.setPosition(targetPosition);
        }).until(() -> atTargetPosition());
    }

    public Command setManual(double percentOutput) {
        return run(() -> motor.setPercentOutput(percentOutput));
    }

    public Command deploy() {
        return this.setPosition(kPivotIntakeRotations);
    }

    public Command retract() {
        return this.setPosition(kPivotStowRotations);
    }
}