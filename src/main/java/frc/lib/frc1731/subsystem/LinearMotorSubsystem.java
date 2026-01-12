package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1678.sim.LinearSim;
import frc.lib.frc1678.sim.LinearSim.LinearSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorIO;
import frc.robot.Robot;

public abstract class LinearMotorSubsystem<M extends MotorIO> extends BaseSubsystem {
    protected M motor = null; // The primary motor controller
    private LinearSim sim = null; // The simulated model of the subsystem 
    private PIDController simPID = null; // The active PIDController for simulating the subsystem
    private SimpleMotorFeedforward simFF = null; // The feedforward component for the sim PID controller

    private Distance targetPosition = Inches.zero();
    private Distance epsilon = Inches.zero(); // Tolerance for position control

    private double gearRatioScalar = 1.0; // Gear ratio reduction (i.e 3:1 means 3 input rotations per output rotation)
    // private DistanceAngleConverter converter = null; // Converter between motor rotations and mechanism distance

    public LinearMotorSubsystem(boolean enabled, Distance drumRadius, double gearRatio) {
        super(enabled);
        this.gearRatioScalar = gearRatio;
        if (!isEnabled()) return;
        configureHardware();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (sim != null) {
            sim.simulate();
        }
    }

    protected abstract void configureHardware(); // Initilizes and configures all motors/sensors

    protected void withSimulation(LinearSimConstants constants, PIDGains simGains) {
        this.sim = new LinearSim(constants);
        this.simPID = simGains.toPIDController();
        this.simFF = new SimpleMotorFeedforward(
            simGains.kS,
            simGains.kV,
            simGains.kA
        );
    }

    protected Distance toMechanism(Angle motorRotations) {
        return Meters.of(motorRotations.div(gearRatioScalar).in(Rotations));
    }

    protected Angle toMotor(Distance mechHeight) {
        return Rotations.of(mechHeight.times(gearRatioScalar).in(Meters));
    }

    protected void setTolerance(Distance epsilon) {
        this.epsilon = epsilon;
    }

    protected void setTolerance(Angle epsilon) {
        this.epsilon = toMechanism(epsilon);
    }

    protected void setTolerence(double epsilon) {
        this.epsilon = Inches.of(epsilon);
    }

    public Distance getPosition() {
        Angle rots = Robot.isSimulation() ? sim.getPosition() : Rotations.of(motor.getRotations());
        // return converter.toDistance(rots);
        return toMechanism(rots);
    }

    public Distance getTargetPosition() {
        return targetPosition;
    }

    public Angle getTargetMotorPosition() {
        // return converter.toAngle(targetPosition.div(gearRatioScalar));
        return toMotor(targetPosition);
    }

    public LinearVelocity getVelocity() {
        AngularVelocity motorVelocity = Robot.isSimulation() ? sim.getVelocity() : RotationsPerSecond.of(motor.getVelocityRPS());
        // return converter.toDistance(motorVelocity.times(Seconds.of(1d))).per(Second).times(gearRatioScalar);
        return toMechanism(motorVelocity.times(Seconds.of(1d))).per(Second).times(gearRatioScalar);
    }

    public AngularVelocity getMotorVelocity() {
        return Robot.isSimulation() ? sim.getVelocity() : RotationsPerSecond.of(motor.getVelocityRPS());
    }

    public Angle getMotorRotations() {
        return Robot.isSimulation() ? sim.getPosition() : Rotations.of(motor.getRotations());
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

    protected void setPosition(Distance position, int pidSlot) {
        this.targetPosition = position;

        if (Robot.isReal() || Robot.isSimulation() && (sim == null || simPID == null)) {
            this.setMotorRotations(getTargetMotorPosition(), pidSlot);
        } else {
            double desiredVoltage = simPID.calculate(
                getMotorRotations().in(Rotations),
                getTargetMotorPosition().in(Rotations)) +
                    simFF.calculate(getMotorVelocity().in(RotationsPerSecond)
            );
            
            this.sim.setVoltage(Volts.of(desiredVoltage));
        }
    }

    protected void setPosition(Distance position) {
        this.setPosition(position, 0);
    }

    protected void setPosition(double position, int pidSlot) {
        this.setPosition(Meters.of(position), pidSlot);
    }

    protected void setPosition(double position) {
        this.setPosition(position, 0);
    }

    protected boolean atTargetPosition() {
        return getPosition().isNear(targetPosition, epsilon);
    }

    protected boolean atTargetPosition(Distance epsilon) {
        return getPosition().isNear(targetPosition, epsilon);
    }

    protected boolean atPosition(Distance desiredPosition) {
        return getPosition().isNear(desiredPosition, epsilon);
    }

    protected boolean atPosition(Distance desiredPosition, Distance epsilon) {
        return getPosition().isNear(desiredPosition, epsilon);
    }

    protected Command setMotionProfileSpeeds(double velocity, double acceleration) {
        return new InstantCommand(() -> {
            motor.withMotionProfile(velocity, acceleration);
        });
    }

    protected Command setPositionCommand(Distance position) {
        return run(() -> setPosition(position))
        .until(() -> atPosition(position))
        .andThen(stopCommand());
    }

    protected Command setPositionCommand(Angle targetMotorPosition) {
        return this.setPositionCommand(toMechanism(targetMotorPosition));
    }

    protected Command setPositionCommand(Distance targetPosition, Distance epsilon) {
        return run(() -> setPosition(targetPosition))
        .until(() -> atPosition(targetPosition, epsilon))
        .andThen(stopCommand());
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            motor.brake();
            if (sim != null) {
                this.targetPosition = getPosition();
                this.sim.setVoltage(Volts.zero());
            }
        }).withName("Stop");
    }
}