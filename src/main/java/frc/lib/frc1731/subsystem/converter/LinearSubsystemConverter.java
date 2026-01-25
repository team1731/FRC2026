package frc.lib.frc1731.subsystem.converter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LinearSubsystemConverter extends SubsystemConverter<Distance> {
    public LinearSubsystemConverter(double gearRatio) {
        super(gearRatio);
    }

    @Override
    public Angle toMotor(Distance mechanismValue) {
        return Rotations.of(mechanismValue.div(gearRatioScalar * 2 * Math.PI).in(Meters));
    }

    @Override
    public Distance toMechanism(Angle motorValue) {
        return Meters.of(motorValue.times(gearRatioScalar * 2 * Math.PI).in(Rotations));
    }
}