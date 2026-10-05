package frc.lib.frc1731.subsystem;


import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.SmartLogger;
import frc.robot.RobotConstants;

/**
 * Base class for team-owned command subsystems.
 *
 * <p>Provides consistent activation gating, command helpers, telemetry logging, and optional
 * SysId support. Subclasses implement {@link #periodicTelemetry()} instead of overriding
 * {@link #periodic()} directly.
 */
public abstract class BaseSubsystem extends SubsystemBase {
    private SysIdRoutine sysIdRoutine = null;
    private boolean isActive = true;
    protected SmartLogger logger = null;

    /**
     * Abstract super class used to build subsystems more easily with logging, sysID, and more built-in
     * @param nameModifier tag to be added at the start of the subsystem when logging (left vs. right, etc.)
     */
    protected BaseSubsystem(String nameModifier) {
        this.setName(nameModifier + getName());
        this.logger = new SmartLogger(getName(), () -> RobotConstants.kLogTeamOutputs);
    }

    /**
     * Abstract super class used to build subsystems more easily with logging, sysID, and more built-in
     */
    protected BaseSubsystem() {
        this("");
    }

    /**
     * Disables hardware commands and telemetry updates for this subsystem.
     *
     * @return this subsystem, typed as the subclass for fluent construction
     */
    @SuppressWarnings("unchecked")
    public <T extends BaseSubsystem> T deactivate() {
        this.isActive = false;
        return (T) this;
    }

    /**
     * Enables hardware commands and telemetry updates for this subsystem.
     *
     * @return this subsystem, typed as the subclass for fluent construction
     */
    @SuppressWarnings("unchecked")
    public <T extends BaseSubsystem> T activate() {
        this.isActive = true;
        return (T) this;
    }

    /**
     * Whether we want the subsystem's hardware to be commandable
     */
    public boolean isActiveSubsystem() {
        return isActive;
    }

    /**
     * Whether there is a command actively running on this subsystem
     */
    public boolean isCurrentlyCommanded(){
        return super.getCurrentCommand() != null && super.getCurrentCommand() != super.getDefaultCommand();
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     */
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     *
     * @param command to command to set as default
     */
    public void setDefaultCommand(Runnable runnable) {
        this.setDefaultCommand(run(runnable));
    }

    /**
     * Returns the current command that is actively running on this subsystem
     */
    public Command getCurrentCommand() {
        if (super.getCurrentCommand() == null) { // No command currently running
            return Commands.none();
        }
        return super.getCurrentCommand();
    }

    /**
     * Returns the default command for this subsystem
     */
    public Command getDefaultCommand() {
        if (super.getDefaultCommand() == null) { // No default command
            return Commands.none();
        }
        return super.getDefaultCommand();
    }

    /**
     * Sets the SysId routine for this subsystem
     * @param rampRate The quasistatic voltage ramp rate in volts per second (Default: 1 V/s)
     * @param stepRate The dynamic voltage step rate in volts (Default: 7 V)
     * @param timeout The timeout in seconds (Default: 10 s)
     * @param outputConsumer The consumer that applies voltage to the motor being characterized
     */
    protected void initSysId(double rampRate, double stepRate, double timeout, Consumer<Voltage> outputConsumer, Consumer<SysIdRoutineLog> logConsumer) {
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(rampRate).per(Second), Volts.of(stepRate), Seconds.of(timeout)), 
            new SysIdRoutine.Mechanism(outputConsumer,logConsumer, this, getName())
        );
    }

        /**
     * Sets the SysId routine for an angular subsystem with default voltage and ramp parameters
     * @param outputConsumer The consumer that applies voltage to the motor being characterized
     * @param angle The supplier for the current position of the motor
     * @param velocity The supplier for the current velocity of the motor
     * @param voltage The supplier for the voltage applied to the motor
     */
    protected void initSysId(double rampRate, double stepRate, double timeout, 
                                Consumer<Voltage> outputConsumer, Supplier<Angle> angle, 
                                    Supplier<AngularVelocity> velocity, Supplier<Voltage> voltage) {
        this.initSysId(rampRate, stepRate, timeout, outputConsumer, log -> {
            log.motor(getName()).angularPosition(angle.get()).angularVelocity(velocity.get()).voltage(voltage.get());
        });
    }

    /**
     * Sets the SysId routine for an angular subsystem with default voltage and ramp parameters
     * @param outputConsumer The consumer that applies voltage to the motor being characterized
     * @param angle The supplier for the current position of the motor
     * @param velocity The supplier for the current velocity of the motor
     * @param voltage The supplier for the voltage applied to the motor
     */
    protected void initSysId(Consumer<Voltage> outputConsumer, Supplier<Angle> angle, Supplier<AngularVelocity> velocity, Supplier<Voltage> voltage) {
        this.initSysId(1, 7, 10, outputConsumer, log -> {
            log.motor(getName()).angularPosition(angle.get()).angularVelocity(velocity.get()).voltage(voltage.get());
        });
    }

    /**
     * Returns the applied SysId routine for this subsystem
     */
    public SysIdRoutine getsysIdRoutine() {
        return sysIdRoutine;
    }

    /**
     * Runs the dynamic motions for the sysid characterization tool
     * @param forward Whether to run the routine in the forward or reverse direction
     */
    public Command dynamicSysIdCommand(boolean forward) {
        if (sysIdRoutine == null) return Commands.none().withName("DynamicSysId");
        return Commands.either(
            sysIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse),
            Commands.none(),
            () -> isActiveSubsystem() && sysIdRoutine != null
        ).withName("DynamicSysId");
    }
    
    /**
     * Runs the quasistatic motions for the sysid characterization tool
     * @param forward Whether to run the routine in the forward or reverse direction
     */
    public Command quasistaticSysIdCommand(boolean forward) {
        if (sysIdRoutine == null) return Commands.none().withName("QuasistaticSysId");
        return Commands.either(
            sysIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse),
            Commands.none(),
            () -> isActiveSubsystem() && sysIdRoutine != null
        ).withName("QuasistaticSysId");
    }

    /**
     * Creates a run command that only executes when the subsystem is active.
     *
     * @param runnable action to execute while scheduled
     * @return gated command, or no-op when inactive
     */
    @Override
    public Command run(Runnable runnable) {
        return Commands.either(super.run(() -> {
            if (isActiveSubsystem()) runnable.run();
        }).until(() -> !isActiveSubsystem()), Commands.none(), this::isActiveSubsystem);
    }

    /**
     * Creates a one-shot command that only executes when the subsystem is active.
     *
     * @param runnable action to execute once
     * @return gated command, or no-op when inactive
     */
    @Override
    public Command runOnce(Runnable runnable) {
        return Commands.either(super.runOnce(runnable), Commands.none(), () -> isActiveSubsystem());
    }

    /**
     * Creates a run/end command that only executes when the subsystem is active.
     *
     * @param runnable action to execute while scheduled
     * @param end cleanup action to execute when the command ends
     * @return gated command, or no-op when inactive
     */
    @Override
    public Command runEnd(Runnable runnable, Runnable end) {
        return Commands.either(super.runEnd(() -> {
            if (isActiveSubsystem()) runnable.run();
        }, end).until(() -> !isActiveSubsystem()), Commands.none(), this::isActiveSubsystem);
    }

    /**
     * Allows the {@code AutoLogged} inputs to be logged onto advantage scope
     * @param inputs {@code AutoLogged} inputs to be logged
     */
    protected <I extends LoggableInputs> void processInputs(I inputs) {
        logger.processInputs(inputs);
    }

    /**
     * Active loop to write and read logs
     */
    public abstract void periodicTelemetry();

    @Override
    public void periodic() {
        if (isActiveSubsystem()) {
            periodicTelemetry();
            if (logger.isOutputEnabled()) {
                Command current = super.getCurrentCommand();
                Command defaultCommand = super.getDefaultCommand();
                logger.log("Command/Actively Commanded", current != null && current != defaultCommand);
                logger.log("Command/Has Default Command", defaultCommand != null);
                logger.log("Command/Active Command", current == null ? "None" : current.getName());
                logger.log("Command/Default Command", defaultCommand == null ? "None" : defaultCommand.getName());
            }
        }
    }
}
