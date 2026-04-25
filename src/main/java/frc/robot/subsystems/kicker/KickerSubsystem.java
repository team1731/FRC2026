package frc.robot.subsystems.kicker;


import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc6328.LoggedTunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class KickerSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private LoggedTunableNumber kP = new LoggedTunableNumber("Kicker/kP", 1.0, () -> true);
    private LoggedTunableNumber kV = new LoggedTunableNumber("Kicker/kV", 0.12, () -> true);

    public KickerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kBottomtKickerConfig)
            .withFollower(Ports.kToptKickerConfig);
        // motor.withPIDGains(kPIDGains);
        // motor.withStatorCurrentLimit(kCurrentLimit);
        motor.withPIDGains(KickerConstants.kPIDGains);
       // Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
       //     motor.withPIDGains(
       //         new PIDGains()
      //          .setP(kP.get())
       //         .setV(kV.get())
       //     );
      //  }));
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
        return setPercent(0.8);
    }

    public Command eject() {
        return setPercent(-0.5);
    }

    public Command stop() {
        return run(() -> {
            this.motor.coast();
        });
    }
}