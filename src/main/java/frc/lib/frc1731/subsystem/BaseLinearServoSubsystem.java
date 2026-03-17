package frc.lib.frc1731.subsystem;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1678.sim.LinearSim;
import frc.lib.frc1678.sim.LinearSim.LinearSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.SubsystemConfiguration;

public abstract class BaseLinearServoSubsystem<M extends MotorIO> extends BaseSubsystem {
    private M motor;
    private double target = 0;
    private double epsilon = 0.1; // Rotations

    private LinearSim sim = null;
    private ProfiledPIDController simPID = null;
    private ElevatorFeedforward simFF = null;

    public <E extends SubsystemConfiguration> BaseLinearServoSubsystem(String nameModifier, E config, boolean enabled) {
        super(nameModifier, config, enabled);
    }

    public <E extends SubsystemConfiguration> BaseLinearServoSubsystem(E config, boolean enabled) {
        super(config, enabled);
    }

    public BaseLinearServoSubsystem(boolean enabled) {
        super(enabled);
    }

    public void initSimulation(LinearSimConstants constants, PIDGains simGains, double maxVelocity, double maxAcceleration) {
        this.sim = new LinearSim(constants);
        this.simPID = simGains.toProfiledPIDController(maxVelocity, maxAcceleration);
        this.simFF = new ElevatorFeedforward(simGains.kS, simGains.kG, simGains.kV);
    }

    public boolean atTarget(double target, double epsilon) {
        if (Robot.isSimulation() && sim != null) return sim.getPosition().isNear(Rotations.of(target), Rotations.of(epsilon));
        return Rotations.of(motor.getRotations()).isNear(Rotations.of(target), Rotations.of(epsilon));
    }

    public boolean atTarget(double epsilon) {
        return atTarget(target, epsilon);
    }

    public boolean atTarget() {
        return atTarget(target, epsilon);
    }

    public Command setPosition(double position) {
        return run(() -> {
            this.target = position;
            this.motor.setPosition(target);
            if (Robot.isSimulation() && sim != null) {
                double voltage = simPID.calculate(motor.getRotations(), target) + simFF.calculate(target, 0);
                sim.setVoltage(Volts.of(voltage));
            }
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.brake();
            if (Robot.isSimulation() && sim != null) sim.setVoltage(Volts.zero());
        });
    }
}