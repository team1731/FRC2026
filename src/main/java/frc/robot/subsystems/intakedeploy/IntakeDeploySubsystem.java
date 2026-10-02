package frc.robot.subsystems.intakedeploy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.TrapezoidalPositionRequest;
import frc.lib.frc1731.subsystem.BaseAngularServoInputsAutoLogged;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class IntakeDeploySubsystem extends BaseServoSubsystem<MotorIOTalonFX, BaseAngularServoInputsAutoLogged> {
    public static final TrapezoidalPositionRequest kHomeRequest = 
        new TrapezoidalPositionRequest(IntakeDeployConstants.kHomeAngle)
            .withSpeeds(IntakeDeployConstants.kMaxVelocity, IntakeDeployConstants.kMaxAcceleration);

    public static final TrapezoidalPositionRequest kDeployRequest = 
        new TrapezoidalPositionRequest(IntakeDeployConstants.kDeployAngle)
            .withSpeeds(IntakeDeployConstants.kMaxVelocity, IntakeDeployConstants.kMaxAcceleration);
    
    public static final TrapezoidalPositionRequest kJiggleRequest = 
        new TrapezoidalPositionRequest(IntakeDeployConstants.kDeployAngle)
            .withSpeeds(IntakeDeployConstants.kMaxVelocity, IntakeDeployConstants.kMaxAcceleration);

    public static final IntakeDeploySubsystem kInstance = new IntakeDeploySubsystem();

    private IntakeDeploySubsystem() {
        super(IntakeDeployConstants.getIO(), new BaseAngularServoInputsAutoLogged());
    }

    @Override
    protected BaseAngularServoInputsAutoLogged updateInputs(BaseAngularServoInputsAutoLogged inputs) {
        inputs.currentPosition = getPosition();
        inputs.setpointPosition = getSetpoint();
        inputs.atSetpoint = atSetpoint();
        RobotState.updateIntake(inputs.currentPosition);
        return inputs;
    }

    public Command home() {
        return applyRequest(kHomeRequest);
    }

    public Command deploy() {
        return applyRequest(kDeployRequest);
    }

    public Command jiggle() {
        return 
        applyRequest(kJiggleRequest.withPosition(IntakeDeployConstants.kDeployAngle))
            .withTimeout(0.5)
        .andThen(applyRequest(kJiggleRequest.withPosition(IntakeDeployConstants.kHomeAngle))
            .withTimeout(0.5))
        .repeatedly()
        .finallyDo(() -> this.getMotor().setPosition(IntakeDeployConstants.kHomeAngle, 0));
    }
}