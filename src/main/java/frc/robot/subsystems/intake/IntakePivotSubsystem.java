package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.RobotConstants;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

public class IntakePivotSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private CANcoder cancoder;
    private IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

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
            .withRemoteCANcoder(new CoreCANcoder(Ports.kPivotCANcoderId, new CANBus(RobotConstants.kMainCANBus)))
            .withRotorToSensorRatio(kPivotGearRatio)
        );

        motor.withMotionMagicConfigs(
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(2)
            .withMotionMagicAcceleration(2)
        );

        motor.setDynamicMotionMagicSpeeds(2, 2);
        cancoder = new CANcoder(Ports.kPivotCANcoderId);

        CANcoderConfiguration coderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(-0.21435546875));
        cancoder.getConfigurator().apply(coderConfig);
        
        motor.getMotor().setPosition(cancoder.getAbsolutePosition().waitForUpdate(0.2).getValueAsDouble());
    }

    public boolean atTargetPosition() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations(), inputs.targetPosition, kPivotEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentPosition = motor.getRotations();
        inputs.atTargetPosition = Utils.isWithin(inputs.currentPosition, inputs.targetPosition, kPivotEpsilon);
        logger.processInputs(inputs);
    }
    
    private Command setPosition(DoubleSupplier position) {
        return run(() -> {
            inputs.targetPosition = position.getAsDouble();
            motor.setPosition(position.getAsDouble());
        });
    }

    public Command setManual(double percentOutput) {
        return run(() -> motor.setPercentOutput(percentOutput));
    }

    public Command deploy() {
        return this.setPosition(() -> kPivotIntakeRotations);
    }

    public Command retract() {
        return this.setPosition(() -> kPivotStowRotations);
    }

    public void setPosition(double position) {
        motor.setPosition(position);
    }
}