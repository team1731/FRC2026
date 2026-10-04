package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.VelocityRequest;
import frc.lib.frc1731.subsystem.BaseVelocityInputsAutoLogged;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;

public class FlywheelSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX, BaseVelocityInputsAutoLogged> {
    public static final VelocityRequest kShotRequest = new VelocityRequest().withEpsilonThreshold(FlywheelConstants.kEpsilon);

    public FlywheelSubsystem() {
        super(FlywheelConstants.getIO(), new BaseVelocityInputsAutoLogged());
        super.setDefaultCommand(stop());
    }

    @Override
    protected BaseVelocityInputsAutoLogged updateInputs(BaseVelocityInputsAutoLogged inputs) {
        inputs.currentVelocity = getVelocity();
        inputs.setpointVelocity = getSetpoint();
        inputs.atSetpoint = atSetpoint();
        
        logger.log("FlywheelWarmingUp", !getSetpoint().equals(RotationsPerSecond.zero()));
        return inputs;
    }

    public Command shoot(Supplier<AngularVelocity> setpoint) {
        return this.applyRequest(() -> kShotRequest.withVelocity(setpoint.get()));
    }

    public Command shoot(AngularVelocity setpoint) {
        return this.applyRequest(kShotRequest.withVelocity(setpoint));
    }

    public Command warmup() {
        return this.shoot(FlywheelConstants.kWarmupVelocity);
    }
}