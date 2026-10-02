package frc.lib.frc1678.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Class for simulating a pivoting system powerd by one or more motors like a rotating arm.
 */
public class PivotSim extends MechanismSim {
	protected final SingleJointedArmSim sim;
	/**
	 * Creates a PivotSim from provided constants.
	 *
	 * @param constants Constants to use for PivotSim.
	 */
	public PivotSim(PivotSimConstants constants) {
		super(constants.motor, constants.gearing);
		sim = new SingleJointedArmSim(
				constants.motor,
				constants.gearing,
				constants.momentOfInertia.in(Units.KilogramSquareMeters),
				constants.armLength.in(Units.Meters),
				constants.mechanismMinHardStop.in(Units.Radians),
				constants.mechanismMaxHardStop.in(Units.Radians),
				constants.simGravity,
				constants.mechanismStartPos.in(Units.Radians));
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(voltage.in(Units.Volts));
	}

	/**
	 * Constants for creating a PivotSim.
	 */
	public static class PivotSimConstants {
		public DCMotor motor = DCMotor.getKrakenX60(1); // defaults to 1 kraken x60
		public double gearing = 1.0;
		public MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.001);
		public Distance armLength = Meters.one();
		public Angle mechanismMinHardStop = Rotations.of(Double.NEGATIVE_INFINITY);
		public Angle mechanismMaxHardStop = Rotations.of(Double.POSITIVE_INFINITY);
		public boolean simGravity = false;
		public Angle mechanismStartPos = Rotations.zero();

		public PivotSimConstants withMotor(DCMotor motor) {
			this.motor = motor;
			return this;
		}

		public PivotSimConstants withGearing(double gearing) {
			this.gearing = gearing;
			return this;
		}

		public PivotSimConstants withMOI(MomentOfInertia moi) {
			this.momentOfInertia = moi;
			return this;
		}

		public PivotSimConstants withMOI(double moi) {
			this.momentOfInertia = KilogramSquareMeters.of(moi);
			return this;
		}

		public PivotSimConstants withSoftLimits(Angle min, Angle max) {
			this.mechanismMinHardStop = min;
			this.mechanismMaxHardStop = max;
			return this;
		}

		public PivotSimConstants withSoftLimits(double minRotations, double maxRotations) {
			this.mechanismMinHardStop = Rotations.of(minRotations);
			this.mechanismMaxHardStop = Rotations.of(maxRotations);
			return this;
		}

		public PivotSimConstants withStartPosition(Angle position) {
			this.mechanismStartPos = position;
			return this;
		}

		public PivotSimConstants withStartPosition(double position) {
			this.mechanismStartPos = Rotations.of(position);
			return this;
		}

		public PivotSimConstants withGravity(boolean gravity) {
			this.simGravity = gravity;
			return this;
		}
	}

	@Override
	public AngularVelocity getVelocity() {
		return Units.Radians.of(sim.getVelocityRadPerSec()).per(Units.Second);
	}

	@Override
	public Angle getPosition() {
		return Units.Radians.of(sim.getAngleRads());
	}

	@Override
	public Current getStatorCurrent() {
		return Units.Amps.of(sim.getCurrentDrawAmps());
	}

	@Override
	protected void update(Time deltaTime) {
		sim.update(deltaTime.in(Units.Seconds));
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		sim.setState(angle.in(Units.Radians), velocity.in(Units.RadiansPerSecond));
	}
}