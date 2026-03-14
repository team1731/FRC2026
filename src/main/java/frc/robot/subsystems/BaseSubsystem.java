package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.*;

import frc.lib.frc1731.log.SmartLogger;

public abstract class BaseSubsystem extends SubsystemBase {
    private boolean enabled = false;
    protected SmartLogger logger = null;

    private SysIdRoutine sysIdRoutine = null;

    protected BaseSubsystem(boolean enabled) {
        this.enabled = enabled;
        this.logger = new SmartLogger(getName());
    }

    /**
     * Whether we want the subsystem's hardware to be commandable
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Whether there is a command actively running on this subsystem
     */
    public boolean isCurrentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
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
    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }

    /**
     * Runs the dynamic motions for the sysid characterization tool
     * @param forward Whether to run the routine in the forward or reverse direction
     */
    public Command dynamicSysIdCommand(boolean forward) {
        return Commands.either(
            sysIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse),
            Commands.none(),
            () -> isEnabled() && sysIdRoutine != null
        ).withName("DynamicSysId");
    }
    
    /**
     * Runs the quasistatic motions for the sysid characterization tool
     * @param forward Whether to run the routine in the forward or reverse direction
     */
    public Command quasistaticSysIdCommand(boolean forward) {
        return Commands.either(
            sysIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse),
            Commands.none(),
            () -> isEnabled() && sysIdRoutine != null
        ).withName("QuasistaticSysId");
    }

    @Override
    public Command run(Runnable runnable) {
        return Commands.either(super.run(runnable), Commands.none(), () -> isEnabled());
    }

    @Override
    public Command runOnce(Runnable runnable) {
        return Commands.either(super.runOnce(runnable), Commands.none(), () -> isEnabled());
    }

    @Override
    public Command runEnd(Runnable runnable, Runnable end) {
        return Commands.either(super.runEnd(runnable, end), Commands.none(), () -> isEnabled());
    }

    public abstract void periodicTelemetry(); // Periodic logging

    public void periodicOutput() {} // If needed, periodic output to hardware

    @Override
    public void periodic () {
        periodicOutput();
        // periodicTelemetry();
        // logger.log("Command/Actively Commanded", isCurrentlyCommanded());
        // logger.log("Command/Has Default Command", !getDefaultCommand().equals(Commands.none()));
        // logger.log("Command/Active Command", getCurrentCommand().getName());
        // logger.log("Command/Default Command", getDefaultCommand().getName());
    }
}