package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1678.sim.PivotSim;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.SubsystemConfiguration;

public abstract class BasePivotServoSubsystem<M extends MotorIO> extends BaseSubsystem {
    private M motor;
    private Angle target = Rotations.of(0);
    private Angle epsilon = Degrees.of(1);

    private PivotSim sim = null;
    private ProfiledPIDController simPID = null;
    private ArmFeedforward simFF = null;

    public <E extends SubsystemConfiguration> BasePivotServoSubsystem(String nameModifier, E config, boolean enabled) {
        super(nameModifier, config, enabled);
    }

    public <E extends SubsystemConfiguration> BasePivotServoSubsystem(E config, boolean enabled) {
        super(config, enabled);
    }

    public BasePivotServoSubsystem(boolean enabled) {
        super(enabled);
    }

    public void initSimulation(PivotSimConstants constants, PIDGains simGains, double maxVelocity, double maxAcceleration) {
        this.sim = new PivotSim(constants);
        this.simPID = simGains.toProfiledPIDController(maxVelocity, maxAcceleration);
        this.simFF = new ArmFeedforward(simGains.kS, simGains.kG, simGains.kV);
    }

    public boolean atTarget(Angle target, Angle epsilon) {
        if (Robot.isSimulation() && sim != null) return sim.getPosition().isNear(target, epsilon);
        return Rotations.of(motor.getRotations()).isNear(target, epsilon);
    }

    public boolean atTarget(Angle epsilon) {
        return atTarget(target, epsilon);
    }

    public boolean atTarget() {
        return atTarget(target, epsilon);
    }

    public Command setPosition(Angle position) {
        return run(() -> {
            this.target = position;
            this.motor.setPosition(target.in(Rotations));
            if (Robot.isSimulation() && sim != null) {
                double voltage = simPID.calculate(motor.getRotations(), target.in(Rotations)) + simFF.calculate(target.in(Rotations), 0);
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