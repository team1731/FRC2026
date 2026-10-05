package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOSparkMax;
import frc.lib.frc1731.hardware.motor.io.request.PositionRequest;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.RobotState;

public class HopperSubsystem extends BaseServoSubsystem<MotorIOSparkMax> {
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
    public HopperSubsystem() {
        super(HopperConstants.getIO());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentHeight = HopperConstants.kConverter.toDistance(getPosition()).in(Inches);
        inputs.setpointHeight = HopperConstants.kConverter.toDistance(getSetpoint()).in(Inches);
        inputs.hopperExtended = HopperConstants.kConverter.toDistance(getSetpoint()).equals(HopperConstants.kMaxHeight);
        logger.processInputs(inputs);
        RobotState.updateHopper(Inches.of(inputs.currentHeight));
    }

    public Command extend() {
        return applyRequest(new PositionRequest(HopperConstants.kConverter.toAngle(HopperConstants.kMaxHeight)));
    }

    public Command collapse() {
        return applyRequest(new PositionRequest(HopperConstants.kConverter.toAngle(HopperConstants.kHomeHeight)));
    }
}