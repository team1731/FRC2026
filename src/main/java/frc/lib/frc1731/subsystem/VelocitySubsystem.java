package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.lib.frc1731.sim.SimpleVelocitySim;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

public abstract class VelocitySubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M motor; // The primary motor controller
    private AngularVelocity targetVelocity = RotationsPerSecond.zero(); // The setpoint velocity for the motor
    private SimpleVelocitySim sim = null; // The simulated model of the subsystem
    private AngularVelocity epsilon = RotationsPerSecond.of(1); // The velocity tolerance to be considered "at target speed"
    private double mechanismRatio = 1d; // Mechanism input:output reduction ratio (i.e 2:1 ratio means 2 motor rotations per mechanism rotation)
    private AngularVelocity kMaxVelocity = RotationsPerSecond.of(100); // The maximum velocity of the mechanism, used for percent output commands

    protected VelocitySubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        initializeHardware();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (sim != null) sim.periodic();
    }

    protected void withTolerance(AngularVelocity epsilon) {
        this.epsilon = epsilon;
    }

    protected void withMaxVelocity(AngularVelocity maxVelocity) {
        this.kMaxVelocity = maxVelocity;
    }

    protected void withSysId(double quasistaticRampRate, double dynamicStepRate, double timeout) {
        super.initSysId(quasistaticRampRate, dynamicStepRate, timeout,
            volts -> setVoltage(volts),
            () -> getMotorAngle(), 
            () -> getVelocity(), 
            () -> getVoltage()
        );
    }

    protected void withSimulation(SimConstants constants, PIDGains simGains) {
        this.sim = new SimpleVelocitySim(constants.motor, constants.gearing, constants.radius, constants.weight, simGains);
    }

    public Angle getMotorAngle() {
        if (!isEnabled()) return Degrees.of(0);
        if (Robot.isSimulation() && sim != null) return sim.getPosition().div(mechanismRatio);
        return motor != null ? Rotations.of(motor.getRotations()) : Degrees.zero();
    }

    public AngularVelocity getVelocity() {
        if (!isEnabled()) return DegreesPerSecond.of(0);
        if (Robot.isSimulation() && sim != null) return sim.getVelocity().div(mechanismRatio);
        return motor != null ? RotationsPerSecond.of(motor.getVelocityRPS()) : RotationsPerSecond.zero();
    }

    public Voltage getVoltage() {
        if (!isEnabled()) return Volts.of(0);
        if (Robot.isSimulation() && sim != null) return sim.getAppliedVoltage();
        return Volts.of(motor.getAppliedVoltage());
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public boolean atTargetVelocity() {
        return getVelocity().isNear(getTargetVelocity(), epsilon);
    }

    protected void setVoltage(Voltage volts) {
        if (!isEnabled()) return;
        if (motor != null)  motor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation() && sim != null) sim.setVoltage(volts); 
    }

    public Command setPercentOutput(DoubleSupplier percent) {
        return run(() -> {
            this.targetVelocity = kMaxVelocity.times(percent.getAsDouble());
            if (motor != null)  motor.setPercentOutput(percent.getAsDouble());
            if (Robot.isSimulation() && sim != null) sim.setVoltage(Volts.of(percent.getAsDouble() * 12d));
        }).withName("SetPercent");
    }

    public Command setPercentOutput(double percent) {
        return this.setPercentOutput(() -> percent);
    }

    public Command setVelocity(Supplier<AngularVelocity> velocity) {
        return run(() -> {
            this.targetVelocity = velocity.get();
            if (motor != null) motor.setVelocityRPS(targetVelocity.in(RotationsPerSecond) / mechanismRatio);
            if (Robot.isSimulation() && sim != null) sim.setVelocity(targetVelocity);
        }).withName("SetVelocity");
    }

    public Command setVelocity(AngularVelocity velocity) {
        return this.setVelocity(() -> velocity);
    }

    public Command stop() {
        return setPercentOutput(0.0)
        .withName("Stop");
    }
}