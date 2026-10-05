package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.*;

public class KickerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX> {
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
    public KickerSubsystem() {
        super(KickerConstants.getIO());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.setpointVelocity = getSetpoint().in(RotationsPerSecond);
        logger.processInputs(inputs);
    }

    public Command feed() {
        return setVelocity(KickerConstants.kFeedVelocity);
    }

    public Command spit() {
        return setVelocity(KickerConstants.kSpitVelocity);
    }
}