package frc.lib.frc1731.subsystem.converter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;

public abstract class SubsystemConverter<M extends Measure<?>> {
    protected final double gearRatioScalar;
    protected SubsystemConverter(double gearRatio) {
        gearRatioScalar = gearRatio;
    }

    /**
     * Determines the motor position for the given mechanism position
     */
    public abstract Angle toMotor(M mechanismValue);

    /**
     * Determines the mechanism position for the given motor position
     */
    public abstract M toMechanism(Angle motorValue);
}