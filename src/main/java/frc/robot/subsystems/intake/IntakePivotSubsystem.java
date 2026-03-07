package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

public class IntakePivotSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private double targetPosition = 0;

    public IntakePivotSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(IntakeConstants.kPivotMotorConfig);
        motor.getMotor().clearStickyFaults();
        motor.withPIDGains(kPivotGains);
        motor.withStatorCurrentLimit(40d);
        motor.setSoftLimits(kPivotIntakeRotations, kPivotStowRotations);
        motor.withFeedbackConfigs(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(15)
            .withRemoteCANcoder(new CoreCANcoder(15))
            .withRotorToSensorRatio(36d)
        );

        motor.withMotionMagicConfigs(
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(12)
            .withMotionMagicAcceleration(2)
        );

        motor.setDynamicMotionMagicSpeeds(2, 2);    
        
        Robot.IS_ENABLED.onTrue(new InstantCommand(() -> motor.setPercentOutput(0)));
    }

    public boolean atTargetPosition() {
        return Utils.isWithin(motor.getRotations(), targetPosition, 0.01);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", motor.getRotations());
        logger.log("Target Rotations", targetPosition);
        logger.log("At Target Position", atTargetPosition());
    }
    
    private Command setPositionCommand(double position) {
        return run(() -> {
            targetPosition = position;
            motor.setPosition(targetPosition);
        }).until(() -> atTargetPosition());
    }

    public Command setManualCommand(double percentOutput) {
        return run(() -> motor.setPercentOutput(percentOutput));
    }

    public Command deployCommand() {
        return this.setPositionCommand(kPivotIntakeRotations);
    }

    public Command retractCommand() {
        return this.setPositionCommand(kPivotStowRotations);
    }

    public Command unjamCommand() {
        // Runs the motor forward and reverse for short spurts to attempt to unjam the mechanism; Uses recursion?
        return this.setManualCommand(0.2)
            .withTimeout(0.5d)
            .andThen(setManualCommand(-0.2))
            .andThen(unjamCommand());
    }
}