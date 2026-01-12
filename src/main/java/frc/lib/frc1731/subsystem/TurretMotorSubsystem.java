package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;

public abstract class TurretMotorSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M motor = null; // The primary motor controller
    private DCMotorSim sim = null; // The simulated model of the turret subsystem
    private PIDController simPID = null; // The active PIDController for simulating the subsystem
    private SimpleMotorFeedforward simFF = null; // The feedforward component for the sim PID controller

    private Angle targetAngle = Degrees.zero(); // Target angle for the turret
    private Angle epsilon = Degrees.zero(); // Tolerance for position control

    private double gearRatioScalar = 1.0; // Gear ratio reduction (i.e 3:1 means 3 input rotations per output rotation)

    protected TurretMotorSubsystem(boolean enabled, Angle tolerance) {
        super(enabled);
    }

    protected abstract void configureHardware(); // Initilizes and configures all motors/sensors

    protected void withSimulation(double kV, double kA) {
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kV, kA), 
            DCMotor.getKrakenX60(1)
        );
    }

    protected void setAngle(Angle desiredAngle) {
        this.targetAngle = desiredAngle;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (sim != null) {
            sim.update(0.02);
        }
    }

    protected void withGearRatio(double gearRatio) {
        this.gearRatioScalar = gearRatio;
    }

    protected Angle getTolerance() {
        return epsilon;
    }

    protected Angle toTurretAngle(Angle motorAngle) {
        return motorAngle.times(gearRatioScalar);
    }

    protected Angle toMotorAngle(Angle turretAngle) {
        return turretAngle.div(gearRatioScalar);
    }

    protected void setTolerance(Angle epsilon) {
        this.epsilon = epsilon;
    }

    protected void setMotorTolerance(Angle epsilon) {
        this.epsilon = toTurretAngle(epsilon);
    }

    protected void setTolerence(double epsilon) {
        this.setTolerance(Degrees.of(epsilon));
    }

    protected void setMotorTolerence(double epsilon) {
        this.setMotorTolerance(Rotations.of(epsilon));
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getTargetMotorPosition() {
        return toMotorAngle(targetAngle);
    }

    public Angle getTurretAngle() {
        if (motor == null) return Degrees.zero();
        return Robot.isSimulation() && sim != null ? sim.getAngularPosition() : 
            toTurretAngle(Rotations.of(motor.getRotations()));
    }

    public Angle getMotorAngle() {
        if (motor == null) return Degrees.zero();
        return Robot.isSimulation() && sim != null ? toMotorAngle(sim.getAngularPosition()) : Rotations.of(motor.getRotations());
    }

    public AngularVelocity getVelocity() {
        if (motor == null) return DegreesPerSecond.zero();
        return Robot.isSimulation() && sim != null ? sim.getAngularVelocity() : 
            toTurretAngle(Rotations.of(motor.getVelocityRPS())).per(Second);
    }

    protected void setMotorRotations(Angle rotations, int pidSlot) {
        motor.setPosition(rotations.in(Rotation), pidSlot);
    }

    protected void setMotorRotations(Angle rotations) {
        this.setMotorRotations(rotations, 0);
    }

    protected void setMotorRotations(double rotations, int pidSlot) {
        this.setMotorRotations(Rotations.of(rotations), pidSlot);
    }

    protected void setMotorRotations(double rotations) {
        this.setMotorRotations(Rotations.of(rotations), 0);
    }

    protected void setTurretAngle(Angle angle, int pidSlot) {
        this.targetAngle = angle;
        Angle targetMotorAngle = toMotorAngle(angle);
        this.setMotorRotations(targetMotorAngle, pidSlot);

        if (sim == null || simPID == null) {
            double desiredVoltage = simPID.calculate(getMotorAngle().in(Rotations), targetMotorAngle.in(Rotations))
                + simFF.calculate(getVelocity().in(RotationsPerSecond));

            this.sim.setInputVoltage(desiredVoltage);
        }
    }

    protected void setTurretAngle(Angle position) {
        this.setTurretAngle(position, 0);
    }

    protected void setTurretAngle(double position, int pidSlot) {
        this.setTurretAngle(Degrees.of(position), pidSlot);
    }

    protected void setTurretAngle(double position) {
        this.setTurretAngle(position, 0);
    }

    protected boolean atTargetAngle() {
        return getTurretAngle().isNear(targetAngle, epsilon);
    }

    protected boolean atTargetAngle(Angle epsilon) {
        return getTurretAngle().isNear(targetAngle, epsilon);
    }

    protected boolean atAngle(Angle desiredAngle) {
        return getTurretAngle().isNear(desiredAngle, epsilon);
    }

    protected boolean atAngle(Angle desiredAngle, Angle epsilon) {
        return getTurretAngle().isNear(desiredAngle, epsilon);
    }

    protected Command setMotionProfileSpeeds(double velocity, double acceleration) {
        return new InstantCommand(() -> {
            motor.withMotionProfile(velocity, acceleration);
        });
    }

    protected Command setTurretAngleCommand(Angle angle) {
        return run(() -> setTurretAngle(angle))
        .until(() -> atAngle(angle));
    }

    protected Command setTurretAngleCommand(Angle angle, Angle epsilon) {
        return run(() -> setTurretAngle(angle))
        .until(() -> atAngle(angle, epsilon));
    }

    protected Command stopCommand() {
        return this.runOnce(() -> {
            motor.brake();
            if (sim != null) sim.setInputVoltage(gearRatioScalar);
        });
    }
}