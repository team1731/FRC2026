package frc.lib.frc1731.subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.hardware.motor.io.request.MotorRequest;

public abstract class BaseMotorSubsystem<IO extends MotorIO> extends BaseSubsystem {
    private IO motor = null;

    public BaseMotorSubsystem(IO motor) {
        this.motor = motor;
    }

    public IO getMotor() {
        return this.motor;
    }

    public Angle getPosition() {
        return motor.getPosition();
    }

    public AngularVelocity getVelocity() {
        return motor.getVelocity();
    }

    public double getMotorVoltage() {
        return motor.getAppliedVoltage();
    }

    /** Compares feedback using the active request's typed absolute tolerance. */
    public boolean atSetpoint() {
        return isActiveSubsystem() && motor.getRequest().atSetpoint(motor);
    }

    /** True when feedback is strictly greater than the active target; epsilon is not applied. */
    public boolean isAbove() {
        return isActiveSubsystem() && motor.getRequest().isAbove(motor);
    }

    /** True when feedback is strictly less than the active target; epsilon is not applied. */
    public boolean isBelow() {
        return isActiveSubsystem() && motor.getRequest().isBelow(motor);
    }

    public Command applyRequest(MotorRequest request) {
        return runOnce(() -> motor.applyRequest(request));
    }

    public Command applyRequest(Supplier<MotorRequest> request) {
        return run(() -> motor.applyRequest(request.get()));
    }

    public Command applyRequestUntilAtSetpoint(MotorRequest request) {
        return Commands.either(
            applyRequest(request).andThen(Commands.waitUntil(this::atSetpoint))
                .until(() -> !isActiveSubsystem()),
            Commands.none(), this::isActiveSubsystem);
    }

    public Command applyRequestUntilAtSetpoint(Supplier<MotorRequest> request) {
        return applyRequest(request).until(this::atSetpoint);
    }

    public Command waitForThenApplyRequest(BooleanSupplier condition, MotorRequest request) {
        return Commands.waitUntil(condition).andThen(this.applyRequest(request));
    }

    public Command waitForThenApplyRequest(BooleanSupplier condition, Supplier<MotorRequest> request) {
        return Commands.waitUntil(condition).andThen(this.applyRequest(request));
    }

    public Command waitForThenApplyRequestUntilAtSetpoint(BooleanSupplier condition, MotorRequest request) {
        return Commands.waitUntil(condition).andThen(applyRequestUntilAtSetpoint(request));
    }

    public Command waitForThenApplyRequestUntilAtSetpoint(BooleanSupplier condition, Supplier<MotorRequest> request) {
        return Commands.waitUntil(condition).andThen(applyRequestUntilAtSetpoint(request));
    }

    @Override
    public void periodicTelemetry() {
        motor.simPeriodic();
    }
}
