package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private RollerSim sim = new RollerSim(kRollerSimConstants);

    private double setpointVelocity = 0.0;

    private PIDController simPIDCtrl = kVelocityGains.toPIDController();
    private SimpleMotorFeedforward simFF = new SimpleMotorFeedforward(kVelocityGains.kS, kVelocityGains.kV);

    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        if (!enabled) return;
        motor = new MotorIOTalonFX(kLeftFlywheelConfig);
        motor.withPIDGains(kVelocityGains);
        SmartDashboard.putNumber("FlywheelSetpoint", setpointVelocity);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", motor.getVelocityRPS());
        logger.log("Target Velocity", setpointVelocity);
        logger.log("Sim Velocity", sim.getVelocity().in(RotationsPerSecond));

        if (sim != null) sim.simulate();
    }

    public boolean atTargetVelocity() {
        return Utils.isWithin(motor.getVelocityRPS(), setpointVelocity, kEpsilon);
    }

    public Command setVelocityCommand(double velocityRPS) {
        return run(() -> {
            if (Robot.isReal()) {
                setpointVelocity = velocityRPS;
                motor.setVelocityRPS(velocityRPS);
            } else if (sim != null){
                double setpointVoltage = simPIDCtrl.calculate(sim.getVelocity().in(RotationsPerSecond), velocityRPS) + simFF.calculate(velocityRPS);
                sim.setVoltage(Volts.of(setpointVoltage));
                logger.log("SetpointVoltage", setpointVoltage);
            }
        }).withName("SetVelocity");
    }

    public Command shootCommand() {
        return run(() -> {
            setpointVelocity = SmartDashboard.getNumber("SetPointVelocity", setpointVelocity);
            motor.setPercentOutput(setpointVelocity);
        }).withName("Shoot");
    }

    public Command stopCommand() {
        return runOnce(() -> {
            setpointVelocity = 0.0;
            motor.setPercentOutput(0.0);
            sim.setVoltage(Volts.zero());
        }).withName("Stop");
    }
}