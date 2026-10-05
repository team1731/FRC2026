package frc.robot.subsystems.intakedeploy;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class IntakeDeploySubsystem extends BaseServoSubsystem<MotorIOTalonFX> {
    private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();
    public IntakeDeploySubsystem() {
        super(IntakeDeployConstants.getIO());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentPosition = getPosition().in(Rotations);
        inputs.setpointPosition = getSetpoint().in(Rotations);
        inputs.deployed = getSetpoint().lt(IntakeDeployConstants.kHomeAngle); // Extended is less than zero so less than
        logger.processInputs(inputs);
        RobotState.updateIntake(getPosition());
    }

    public Command home() {
        return setPositionWithSpeedsAndEpsilon(
            IntakeDeployConstants.kHomeAngle, 
            IntakeDeployConstants.kMaxVelocity, 
            IntakeDeployConstants.kMaxAcceleration,
            IntakeDeployConstants.kEpsilon
        );
    }

    public Command deploy() {
        return setPositionWithSpeedsAndEpsilon(
            IntakeDeployConstants.kDeployAngle, 
            IntakeDeployConstants.kMaxVelocity, 
            IntakeDeployConstants.kMaxAcceleration,
            IntakeDeployConstants.kEpsilon
        );
    }

    public Command jiggle() {
        return home()
            .withTimeout(0.5)
            .andThen(deploy().withTimeout(0.5))
            .repeatedly()
            .finallyDo(() -> this.getMotor().setPosition(IntakeDeployConstants.kHomeAngle, 0));
    }
}