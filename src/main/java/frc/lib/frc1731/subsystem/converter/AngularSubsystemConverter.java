package frc.lib.frc1731.subsystem.converter;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class AngularSubsystemConverter extends SubsystemConverter<Angle>{
    /**
     * @param gearRatio reduction (3:1 means 3 motor rotations per mechanism rotation)
     */
    public AngularSubsystemConverter(double gearRatio) {
        super(gearRatio);
    }
    
    @Override
    public Angle toMotor(Angle mechanism) {
        return mechanism.times(gearRatioScalar);
    }
    
    @Override
    public Angle toMechanism(Angle motor) {
        return motor.div(gearRatioScalar);
    }

    public AngularVelocity toMotor(AngularVelocity mechanism) {
        return mechanism.times(Seconds.of(1)).times(gearRatioScalar).div(Seconds.of(1));
    }

    public AngularVelocity toMechanism(AngularVelocity motor) {
        return motor.times(Seconds.of(1)).div(gearRatioScalar).div(Seconds.of(1));
    }
}