package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.VelocityRequest;
import frc.lib.frc1731.subsystem.*;

import frc.lib.frc1731.subsystem.BaseVelocityInputsAutoLogged;

public class IndexerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX, BaseVelocityInputsAutoLogged> {
    public static final VelocityRequest kFeedRequest = new VelocityRequest(IndexerConstants.kFeedVelocity).withEpsilonThreshold(IndexerConstants.kEpsilon);
    public static final VelocityRequest kSpitRequest = new VelocityRequest(IndexerConstants.kSpitVelocity).withEpsilonThreshold(IndexerConstants.kEpsilon);
    public static final IndexerSubsystem kInstance = new IndexerSubsystem();

    private IndexerSubsystem() {
        super(IndexerConstants.getIO(), new BaseVelocityInputsAutoLogged());
    }

    @Override
    protected BaseVelocityInputsAutoLogged updateInputs(BaseVelocityInputsAutoLogged inputs) {
        inputs.currentVelocity = getVelocity();
        inputs.setpointVelocity = getSetpoint();
        inputs.atSetpoint = atSetpoint();
        return inputs;
    }

    public Command feed() {
        return applyRequest(kFeedRequest);
    }

    public Command spit() {
        return applyRequest(kSpitRequest);
    }
}