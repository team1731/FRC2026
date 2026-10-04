package frc.robot.subsystems.hood;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.TrapezoidalPositionRequest;
import frc.lib.frc1731.subsystem.BaseAngularServoInputsAutoLogged;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class HoodSubsystem extends BaseServoSubsystem<MotorIOTalonFX, BaseAngularServoInputsAutoLogged> {
    // home() uses kShotRequest while constructing the singleton.
    public static final TrapezoidalPositionRequest kShotRequest = new TrapezoidalPositionRequest(
        HoodConstants.kMaxVelocity, 
        HoodConstants.kMaxAcceleration
    ).withEpsilonThreshold(HoodConstants.kEpsilon);

    public HoodSubsystem() {
        super(HoodConstants.getIO(), new BaseAngularServoInputsAutoLogged());
        super.setDefaultCommand(home());
    }

    @Override
    protected BaseAngularServoInputsAutoLogged updateInputs(BaseAngularServoInputsAutoLogged inputs) {
        inputs.currentPosition = getPosition();
        inputs.setpointPosition = getSetpoint();
        inputs.atSetpoint = atSetpoint();
        RobotState.updateHood(inputs.currentPosition);
        return inputs;
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
