package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.sim.SimpleVelocitySim;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

public  class IndexerSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected MotorIO rightMotor; // The primary motor controller
    protected MotorIO leftMotor;
    private AngularVelocity targetVelocity = RotationsPerSecond.zero(); // The setpoint velocity for the motor
    private SimpleVelocitySim sim = null; // The simulated model of the subsystem
    private AngularVelocity epsilon = RotationsPerSecond.of(1); // The velocity tolerance to be considered "at target speed"
    private double mechanismRatio = 1d; // Mechanism input:output reduction ratio (i.e 2:1 ratio means 2 motor rotations per mechanism rotation)
    private AngularVelocity kMaxVelocity = RotationsPerSecond.of(100); // The maximum velocity of the mechanism, used for percent output commands

    public IndexerSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        initializeHardware();
    }

    @Override
    protected void initializeHardware() {
        rightMotor = new MotorIOTalonFX(Ports.kRightIndexerConfig);
        rightMotor.withPIDGains(kPIDGains);

        leftMotor = new MotorIOTalonFX(Ports.kLeftIndexerConfig);
        leftMotor.withPIDGains(kPIDGains);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
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
        return leftMotor != null ? Rotations.of(leftMotor.getRotations()) : Degrees.zero();
    }

    public AngularVelocity getVelocity() {
        if (!isEnabled()) return DegreesPerSecond.of(0);
        if (Robot.isSimulation() && sim != null) return sim.getVelocity().div(mechanismRatio);
        return leftMotor != null ? RotationsPerSecond.of(leftMotor.getVelocityRPS()) : RotationsPerSecond.zero();
    }

    public Voltage getVoltage() {
        if (!isEnabled()) return Volts.of(0);
        if (Robot.isSimulation() && sim != null) return sim.getAppliedVoltage();
        return Volts.of(leftMotor.getAppliedVoltage());
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public boolean atTargetVelocity() {
        return getVelocity().isNear(getTargetVelocity(), epsilon);
    }

    protected void setVoltage(Voltage volts) {
        if (!isEnabled()) return;
        if (leftMotor != null)  leftMotor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation() && sim != null) sim.setVoltage(volts); 
    }

    public Command setPercentOutput(DoubleSupplier percent) {
        return run(() -> {
            this.targetVelocity = kMaxVelocity.times(percent.getAsDouble());
            if (leftMotor != null)  leftMotor.setPercentOutput(percent.getAsDouble());
            if (rightMotor != null)  rightMotor.setPercentOutput(percent.getAsDouble());
            if (Robot.isSimulation() && sim != null) sim.setVoltage(Volts.of(percent.getAsDouble() * 12d));
        }).withName("SetPercent");
    }

    public Command setPercentOutput(double percent) {
        return this.setPercentOutput(() -> percent);
    }

    public Command setVelocity(Supplier<AngularVelocity> velocity) {
        return run(() -> {
            this.targetVelocity = velocity.get();
            if (leftMotor != null)  leftMotor.setPercentOutput(((targetVelocity.in(RotationsPerSecond) / mechanismRatio))/100);
            if (rightMotor != null)  rightMotor.setPercentOutput(((targetVelocity.in(RotationsPerSecond) / mechanismRatio))/100);
         //   if (leftMotor != null) leftMotor.setVelocityRPS(targetVelocity.in(RotationsPerSecond) / mechanismRatio);
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