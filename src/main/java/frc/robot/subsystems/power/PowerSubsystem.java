package frc.robot.subsystems.power;

import static edu.wpi.first.units.Units.Amps;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.subsystem.BaseSubsystem;

/** Real PDH/PDP monitoring and simulated battery loads. */
public class PowerSubsystem extends BaseSubsystem implements AutoCloseable {
    private final PowerDistribution pdp;
    private final PDPSim pdpSim;
    private record SimLoad(String name, DoubleSupplier current) {}
    private final List<SimLoad> loads = new ArrayList<>();

    private final PowerIOInputsAutoLogged inputs = new PowerIOInputsAutoLogged();

    public PowerSubsystem() {
        this(new PowerDistribution());
    }

    /** Use this constructor for a non-default CAN ID or device type. */
    public PowerSubsystem(PowerDistribution pdp) {
        this.pdp = java.util.Objects.requireNonNull(pdp);
        LoggedPowerDistribution.getInstance(pdp.getModule(), pdp.getType());
        pdpSim = RobotBase.isSimulation() ? new PDPSim(pdp) : null;
    }

    /** Registers a virtual simulation channel, not a physical breaker channel. */
    public void addSimLoad(String name, DoubleSupplier supplyCurrent) {
        if (loads.size() >= pdp.getNumChannels()) {
            throw new IllegalArgumentException("Too many simulated power loads");
        }
        loads.add(new SimLoad(java.util.Objects.requireNonNull(name),
            java.util.Objects.requireNonNull(supplyCurrent)));
    }

    /**
     * Uses the entire mechanism model once, including motors already in its DCMotor model.
     * Without a model, falls back to the controller's simulated supply-current reading.
     */
    public void addSimMotor(String name, MotorIO motor) {
        java.util.Objects.requireNonNull(motor);
        addSimLoad(name, () -> motor.getMechSim() == null
            ? motor.getSupplyCurrent()
            : motor.getMechSim().getStatorCurrent().in(Amps)
                * Math.min(1.0, Math.abs(motor.getAppliedDutyCycle())));
    }

    /** Called from Robot.simulationPeriodic(), after all mechanism updates. */
    public void updateSimulation() {
        if (pdpSim == null) return;
        double total = 0.0;
        for (int channel = 0; channel < pdp.getNumChannels(); channel++) {
            double current = channel < loads.size() ? loads.get(channel).current().getAsDouble() : 0.0;
            // This battery model excludes regeneration and invalid sensor values.
            current = Double.isFinite(current) ? Math.max(0.0, current) : 0.0;
            pdpSim.setCurrent(channel, current);
            total += current;
            if (channel < loads.size()) {
                logger.log("SimLoads/" + loads.get(channel).name() + "/SupplyCurrentAmps", current);
            }
        }
        double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(total);
        pdpSim.setVoltage(voltage);
        RoboRioSim.setVInVoltage(voltage);
    }

    public double getTotalCurrent() { return pdp.getTotalCurrent(); }

    @Override
    public void periodicTelemetry() {
        inputs.totalCurrentDraw = getTotalCurrent();
        inputs.batteryVoltage = RobotController.getBatteryVoltage();
        inputs.totalPowerDraw = pdp.getVoltage() * getTotalCurrent();
        inputs.isBrowningOut = RobotController.isBrownedOut();
        inputs.busOffCount = RobotController.getCANStatus().busOffCount;
        logger.processInputs(inputs);
    }

    @Override
    public void close() { pdp.close(); }
}
