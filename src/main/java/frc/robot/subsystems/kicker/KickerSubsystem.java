package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.VelocityRequest;
import frc.lib.frc1731.subsystem.*;

public class KickerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX, BaseVelocityInputsAutoLogged> {
    public static final VelocityRequest kFeedRequest = new VelocityRequest(KickerConstants.kFeedVelocity).withEpsilonThreshold(KickerConstants.kEpsilon);
    public static final VelocityRequest kSpitRequest = new VelocityRequest(KickerConstants.kSpitVelocity).withEpsilonThreshold(KickerConstants.kEpsilon);

    public KickerSubsystem() {
        super(KickerConstants.getIO(), new BaseVelocityInputsAutoLogged());
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