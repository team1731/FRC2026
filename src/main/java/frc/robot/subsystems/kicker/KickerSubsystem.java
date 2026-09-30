package frc.robot.subsystems.kicker;


import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;

public class KickerSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public KickerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(KickerConstants.kCurrentLimit).withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(KickerConstants.kSupplyCurrentLimit).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(KickerConstants.kSupplyCurrentLimit).withSupplyCurrentLowerTime(0);
        motor = new MotorIOTalonFX(Ports.kBottomtKickerConfig)
            .withFollower(limits, Ports.kToptKickerConfig);
        motor.withPIDGains(KickerConstants.kPIDGains);
        motor.withCurrentLimits(limits);
    }

    public double getTipSpeedMPS() {
        if (!isEnabled()) return 0;
        return motor.getVelocityRPS() / KickerConstants.kGearRatio
            * Math.PI * edu.wpi.first.math.util.Units.inchesToMeters(KickerConstants.kRollerDiameter);
    }

    /** Commands roller surface speed in meters per second, using motor-RPS feedback. */
    public Command setTipSpeedMPS(DoubleSupplier tipSpeed) {
        return setVelocity(() -> tipSpeed.getAsDouble() * KickerConstants.kGearRatio
            / (Math.PI * edu.wpi.first.math.util.Units.inchesToMeters(KickerConstants.kRollerDiameter)));
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = motor.getVelocityRPS();
        inputs.atTargetVelocity = Utils.isWithin(inputs.currentVelocity, inputs.targetVelocity, 1);
        logger.processInputs(inputs);
    }

    public Command setPercent(double setpoint) {
        return run(() -> {
            inputs.targetVelocity = setpoint * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM / 60.0;
            this.motor.setPercentOutput(setpoint);
        });
    }

    public Command setVelocity(double setpoint) {
        return run(() -> {
            inputs.targetVelocity = setpoint;
            this.motor.setVelocityRPS(inputs.targetVelocity);
        });
    }

    public Command setVelocity(DoubleSupplier setpoint) {
        return run(() -> {
            inputs.targetVelocity = setpoint.getAsDouble();
            this.motor.setVelocityRPS(inputs.targetVelocity);
        });
    }

    public Command feed() {
        return setVelocity(KickerConstants.kFeedRPS);
        // return setPercent(0.6);
    }

    public Command eject() {
        return setVelocity(KickerConstants.kEjectRPS);
    }

    public Command stop() {
        return run(() -> {
            inputs.targetVelocity = 0;
            this.motor.coast();
        });
    }
}
