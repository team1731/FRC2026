package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class HoodSubsystem extends BaseServoSubsystem<MotorIOTalonFX> {
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
        return setPositionWithSpeedsAndEpsilon(setpoint, HoodConstants.kMaxVelocity,
            HoodConstants.kMaxAcceleration, HoodConstants.kEpsilon);
    }

    public Command setAngle(Angle setpoint) {
        return setAngle(() -> setpoint);
    }

    public Command home() {
        return setAngle(HoodConstants.kHomeAngle).withName("HomeHood");
    }
}
