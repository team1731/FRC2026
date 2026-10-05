package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.TrapezoidalPositionRequest;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class HoodSubsystem extends BaseServoSubsystem<MotorIOTalonFX> {
    public static final TrapezoidalPositionRequest kShotRequest = new TrapezoidalPositionRequest(
        HoodConstants.kMaxVelocity, 
        HoodConstants.kMaxAcceleration
    ).withEpsilonThreshold(HoodConstants.kEpsilon);

    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public HoodSubsystem() {
        super(HoodConstants.getIO());
        super.setDefaultCommand(home());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentRotations = getPosition().in(Rotations);
        inputs.setpointRotations = getSetpoint().in(Rotations);
        inputs.atSetpoint = atSetpoint();
        logger.processInputs(inputs);
        RobotState.updateHood(getPosition());
    }
    
    public Command setAngle(Supplier<Angle> setpoint) {
        return applyRequest(() -> kShotRequest.withPosition(setpoint.get()));
    }

    public Command setAngle(Angle setpoint) {
        return applyRequest(kShotRequest.withPosition(setpoint));
    }

    public Command home() {
        return applyRequest(kShotRequest.withPosition(HoodConstants.kHomeAngle));
    }
}
