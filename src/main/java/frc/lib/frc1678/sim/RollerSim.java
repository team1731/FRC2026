package frc.lib.frc1678.sim;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Class for simulating a rolling system powerd by one or more motors like a shooter.
 */
public class RollerSim extends MechanismSim {
	protected final FlywheelSim sim;
	private double m_positionRad = 0.0; // unbounded, just needs to keep integrating

	/**
	 * Creates a RollerSim from provided constants.
	 *
	 * @param constants Constants to use for RollerSim.
	 */
	public RollerSim(RollerSimConstants constants) {
		super(constants.motor, constants.gearing);
		sim = new FlywheelSim(
				LinearSystemId.createFlywheelSystem(constants.motor, constants.momentOfInertia, constants.gearing),
				constants.motor);
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(voltage.in(Units.Volts));
	}

	/**
	 * Constants for creating a RollerSim.
	 */
	public static class RollerSimConstants {
		public DCMotor motor = DCMotor.getKrakenX60(1); // On default set to single kraken
		public double gearing = 1.0; // On default set to no ratio applied
		public double momentOfInertia = 0.001; // On default set to essentially free spinning

		public RollerSimConstants withMotor(DCMotor motor) {
			this.motor = motor;
			return this;
		}

		public RollerSimConstants withGearing(double gearing) {
			this.gearing = gearing;
			return this;
		}

		public RollerSimConstants withMOI(double momentOfInertia) {
			this.momentOfInertia = momentOfInertia;
			return this;
		}

		public RollerSim build() {
			return new RollerSim(this);
		}
	}

	@Override
	public AngularVelocity getVelocity() {
		return Units.Rotations.of(sim.getAngularVelocityRPM()).per(Units.Minute);
	}

	@Override
	public Angle getPosition() {
		return Units.Radians.of(m_positionRad);
	}

	@Override
	public Current getStatorCurrent() {
		return Units.Amps.of(sim.getCurrentDrawAmps());
	}

	@Override
	protected void update(Time deltaTime) {
		sim.update(deltaTime.in(Units.Seconds));
		m_positionRad += sim.getAngularVelocityRadPerSec() * deltaTime.in(Units.Seconds);
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		sim.setAngularVelocity(velocity.in(Units.RadiansPerSecond));
		m_positionRad = angle.in(Units.Radians);
	}
}