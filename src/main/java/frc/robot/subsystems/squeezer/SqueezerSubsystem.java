package frc.robot.subsystems.squeezer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.frc1731.Utils;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

public class SqueezerSubsystem extends BaseSubsystem {
    private SparkMax motor;
    private SparkClosedLoopController ctrl;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SqueezerIOInputsAutoLogged inputs = new SqueezerIOInputsAutoLogged();

    public SqueezerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new SparkMax(Ports.kSqueezerConfig.kPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.inverted(Ports.kSqueezerConfig.kInverted);

        config.smartCurrentLimit(20);
        config.secondaryCurrentLimit(20);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.5);

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(45);

        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);

        motor.getEncoder().setPosition(0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        ctrl = motor.getClosedLoopController();
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentRotations = motor.getEncoder().getPosition();
        inputs.atTarget = Utils.isWithin(inputs.currentRotations, inputs.targetRotations, 1);
        logger.processInputs(inputs);
    }
    
    public Command raise() {
        return run(() -> {
            inputs.targetRotations = 45;
            ctrl.setSetpoint(inputs.targetRotations, ControlType.kPosition);
        }).withName("Raise");
    }

    public Command squeeze() {
        return run(() -> {
            inputs.targetRotations = 0;
            ctrl.setSetpoint(inputs.targetRotations, ControlType.kPosition);
        }).withName("Squeeze");
    }

    public Command reset() {
        return new InstantCommand(() -> {
                inputs.targetRotations = 0;
                config.softLimit.reverseSoftLimitEnabled(false);
                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }).andThen(
            run(() -> {
                motor.getEncoder().setPosition(0);
                motor.set(-0.1);
            })
        ).finallyDo(() -> {
            config.softLimit.reverseSoftLimitEnabled(true);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    public Command stop() {
        return run(() -> {
            motor.set(0);
        }).withName("Stop");
    }
}