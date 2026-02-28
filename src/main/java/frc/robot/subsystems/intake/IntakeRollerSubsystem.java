package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;
import frc.lib.frc1731.subsystem.VelocitySubsystem;

public class IntakeRollerSubsystem extends VelocitySubsystem<MotorIOTalonFX>{
    public IntakeRollerSubsystem(boolean enabled){
        super(enabled);
        this.withSimulation(new SimConstants(DCMotor.getKrakenX60(1), 1.5, Inches.of(1), Pounds.of(0.1)), kPIDGains);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(kRollerMotorConfig);
        motor.withPIDGains(kPIDGains);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
    }
}
