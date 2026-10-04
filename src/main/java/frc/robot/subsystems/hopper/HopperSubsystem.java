package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOSparkMax;
import frc.lib.frc1731.hardware.motor.io.request.LinearPositionRequest;
import frc.lib.frc1731.subsystem.BaseLinearServoInputsAutoLogged;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class HopperSubsystem extends BaseServoSubsystem<MotorIOSparkMax, BaseLinearServoInputsAutoLogged> {
    public static final LinearPositionRequest kPositionRequest = new LinearPositionRequest(HopperConstants.kConverter).withEpsilonThreshold(HopperConstants.kEpsilon);

    public HopperSubsystem() {
        super(HopperConstants.getIO(), HopperConstants.kConverter, new BaseLinearServoInputsAutoLogged());
    }

    @Override
    protected BaseLinearServoInputsAutoLogged updateInputs(BaseLinearServoInputsAutoLogged inputs) {
        inputs.currentPosition = getLinearPosition();
        inputs.setpointPosition = getLinearSetpoint();
        inputs.atSetpoint = atSetpoint();
        RobotState.updateHopper(inputs.currentPosition);
        return inputs;
    }

    public Command extend() {
        return applyRequest(kPositionRequest.withPosition(HopperConstants.kMaxHeight));
    }

    public Command collapse() {
        return applyRequest(kPositionRequest.withPosition(HopperConstants.kHomeHeight));
    }
}