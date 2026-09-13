package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.VelocitySubsystem;
import frc.robot.Ports;

public class IntakeRollerSubsystem extends VelocitySubsystem<MotorIOTalonFX>{
    private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public IntakeRollerSubsystem(boolean enabled){
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIntakeRollerConfig);
        motor.withPIDGains(kRollerGains);
        motor.withStatorCurrentLimit(kRollerCurrentLimit);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.targetVelocity = getTargetVelocity().in(RotationsPerSecond);
       // logger.processInputs(inputs);

       SmartDashboard.putNumber("INTAKE RPS", inputs.currentVelocity);
    }

    public Command intake() {
        return setVelocity(RotationsPerSecond.of(kRollerIntakeRPS));
        // return setPercentOutput(90);
    }

    public Command eject() {
        return setVelocity(RotationsPerSecond.of(kRollerEjectRPS));
    }
}
