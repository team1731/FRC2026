package frc.lib.frc1731;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1678.util.Util.DistanceAngleConverter;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.hardware.motor.io.request.*;
import frc.lib.frc1731.subsystem.BaseServoSubsystem;
import frc.robot.commands.ContinuousConditionalCommand;

class CommandRequestTest {
    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }
    @AfterEach void cleanup() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    @Test void fixedRequestWaitsForFeedbackAndOwnsSubsystem() {
        Servo servo = new Servo();
        Command command = servo.applyRequestUntilAtSetpoint(new TrapezoidalPositionRequest(Radians.of(2)));
        assertTrue(command.getRequirements().contains(servo));
        command.initialize();
        command.execute();
        assertFalse(command.isFinished());
        servo.getMotor().position = Radians.of(2);
        command.execute();
        assertTrue(command.isFinished());
        command.end(false);
    }

    @Test void oldSetpointCannotBypassWaitCondition() {
        for (boolean supplied : new boolean[] {false, true}) {
            Servo servo = new Servo();
            servo.getMotor().applyRequest(new TrapezoidalPositionRequest(Radians.zero()));
            AtomicBoolean ready = new AtomicBoolean(false);
            MotorRequest request = new TrapezoidalPositionRequest(Radians.of(3));
            Command command = supplied
                ? servo.waitForThenApplyRequestUntilAtSetpoint(ready::get, () -> request)
                : servo.waitForThenApplyRequestUntilAtSetpoint(ready::get, request);
            command.initialize();
            command.execute();
            assertFalse(command.isFinished());
            assertEquals(0, servo.getMotor().getRequestSetpointAsDouble());
            ready.set(true);
            command.execute();
            command.execute();
            assertEquals(3, servo.getMotor().getRequestSetpointAsDouble());
            assertFalse(command.isFinished());
            servo.getMotor().position = Radians.of(3);
            command.execute();
            if (!command.isFinished()) command.execute();
            assertTrue(command.isFinished());
            command.end(false);
        }
    }

    @Test void holdCapturesPositionAtSchedulingAndOnReuse() {
        Servo servo = new Servo();
        Command hold = servo.stop();
        for (double angle : new double[] {2, 4}) {
            servo.getMotor().position = Radians.of(angle);
            hold.initialize();
            hold.execute();
            hold.execute();
            assertEquals(angle, servo.getMotor().getRequestSetpointAsDouble());
            servo.getMotor().position = Radians.of(9);
            hold.execute();
            assertEquals(angle, servo.getMotor().getRequestSetpointAsDouble());
            hold.end(true);
        }
    }

    @Test void inactiveRequestsFinishAndDeactivationRunsCleanupWithoutMoreOutput() {
        Servo servo = new Servo();
        servo.deactivate();
        Command wait = servo.applyRequestUntilAtSetpoint(new TrapezoidalPositionRequest(Radians.of(2)));
        wait.initialize(); wait.execute();
        assertTrue(wait.isFinished());
        wait.end(false);
        assertEquals(0, servo.getMotor().getRequestSetpointAsDouble());
        servo.activate();
        int[] counts = new int[2];
        Command running = servo.runEnd(() -> counts[0]++, () -> counts[1]++);
        running.initialize(); running.execute();
        servo.deactivate(); running.execute();
        assertTrue(running.isFinished());
        running.end(false);
        assertArrayEquals(new int[] {1, 1}, counts);
    }

    @Test void profilesPreserveExistingConstraintsUnlessExplicitlySupplied() {
        FakeMotor motor = new FakeMotor();
        new TrapezoidalPositionRequest(Radians.of(2)).apply(motor);
        DistanceAngleConverter converter = new DistanceAngleConverter(Meters.of(0.5));
        new LinearTrapezoidalPositionRequest(Meters.of(1), converter).apply(motor);
        assertEquals(0, motor.profileUpdates);
        assertEquals(2, motor.target.in(Radians));
        new LinearTrapezoidalPositionRequest(Meters.of(1), MetersPerSecond.of(2),
            MetersPerSecondPerSecond.of(3), converter).withSlot(2).apply(motor);
        assertEquals(1, motor.profileUpdates);
        assertEquals(4, motor.velocity.in(RadiansPerSecond));
        assertEquals(6, motor.acceleration.in(RadiansPerSecondPerSecond));
        assertEquals(2, motor.slot);
    }

    @Test void commandsKeepIndependentTargetsAndMissingSysIdIsSafe() {
        Servo servo = new Servo();
        Command first = servo.setPositionWithSpeedsAndEpsilon(Radians.of(2),
            RadiansPerSecond.of(1), RadiansPerSecondPerSecond.of(2), Radians.of(0.1));
        Command second = servo.setPositionWithSpeedsAndEpsilon(Radians.of(4),
            RadiansPerSecond.of(1), RadiansPerSecondPerSecond.of(2), Radians.of(0.1));
        first.initialize(); first.execute();
        assertEquals(2, servo.getMotor().getRequestSetpointAsDouble());
        first.end(true);
        second.initialize(); second.execute();
        assertEquals(4, servo.getMotor().getRequestSetpointAsDouble());
        second.end(true);
        first.initialize(); first.execute();
        assertEquals(2, servo.getMotor().getRequestSetpointAsDouble());
        first.end(true);
        assertDoesNotThrow(() -> servo.dynamicSysIdCommand(true));
        assertDoesNotThrow(() -> servo.quasistaticSysIdCommand(false));
    }

    @Test void conditionalSwitchesBranchesAndReservesBothRequirements() {
        Subsystem a = new SubsystemBase() {};
        Subsystem b = new SubsystemBase() {};
        Branch yes = new Branch(a), no = new Branch(b);
        AtomicBoolean condition = new AtomicBoolean(true);
        Command command = new ContinuousConditionalCommand(yes, no, condition::get);
        assertTrue(command.getRequirements().containsAll(java.util.Set.of(a, b)));
        command.initialize(); command.execute();
        condition.set(false); command.execute();
        assertEquals(1, yes.interruptions);
        assertEquals(1, no.starts);
        no.finished = true; command.execute(); command.execute();
        assertEquals(1, no.completions);
        assertEquals(2, no.executions);
        assertFalse(command.isFinished());
        condition.set(true); command.execute();
        assertEquals(2, yes.starts);
        command.end(true);
        assertEquals(2, yes.interruptions);
    }

    static class Branch extends Command {
        int starts, executions, interruptions, completions;
        boolean finished;
        Branch(Subsystem requirement) { addRequirements(requirement); }
        @Override public void initialize() { starts++; }
        @Override public void execute() { executions++; }
        @Override public boolean isFinished() { return finished; }
        @Override public void end(boolean interrupted) {
            if (interrupted) interruptions++; else completions++;
        }
    }
    static class Servo extends BaseServoSubsystem<FakeMotor> {
        Servo() { super(new FakeMotor()); }
        @Override public void periodicTelemetry() {}
    }
    static class FakeMotor extends MotorIO {
        Angle position = Radians.zero(), target = Radians.zero();
        AngularVelocity velocity;
        AngularAcceleration acceleration;
        int profileUpdates, slot;
        FakeMotor() { super(MotorConstants.kKrakenX60, null); }
        @Override public <M> M getMotor() { return null; }
        @Override public <IO extends MotorIO> IO withSimulation(MechanismSim sim) { return null; }
        @Override public void resetEncoderPosition(Angle value) { position = value; }
        @Override public void updateTrapezoidalSpeeds(AngularVelocity v, AngularAcceleration a, int s) {
            velocity = v; acceleration = a; slot = s; profileUpdates++;
        }
        @Override public Angle getPosition() { return position; }
        @Override public AngularVelocity getVelocity() { return RadiansPerSecond.zero(); }
        @Override public double getAppliedVoltage() { return 0; }
        @Override public double getAppliedDutyCycle() { return 0; }
        @Override public double getSupplyCurrent() { return 0; }
        @Override public double getStatorCurrent() { return 0; }
        @Override public Angle getForwardLimit() { return Radians.zero(); }
        @Override public Angle getReverseLimit() { return Radians.zero(); }
        @Override public void coast() {}
        @Override public void brake() {}
        @Override public void setPercent(double value) {}
        @Override public void setVoltage(double value) {}
        @Override public void setPosition(Angle value, int s) { target = value; slot = s; }
        @Override public void setPositionTrapezoidal(Angle value, int s) { target = value; slot = s; }
        @Override public void setVelocity(AngularVelocity value, int s) {}
        @Override public void simPeriodic() {}
    }
}
