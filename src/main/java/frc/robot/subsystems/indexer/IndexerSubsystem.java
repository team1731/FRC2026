package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.*;

public class IndexerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX> {
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    public IndexerSubsystem() {
        super(IndexerConstants.getIO());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.setpointVelocity = getSetpoint().in(RotationsPerSecond);
        logger.processInputs(inputs);
    }

    public Command feed() {
        return setVelocityWithEpsilon(IndexerConstants.kFeedVelocity, IndexerConstants.kEpsilon);
    }

    public Command spit() {
        return setVelocityWithEpsilon(IndexerConstants.kSpitVelocity, IndexerConstants.kEpsilon);
    }
}