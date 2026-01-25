package frc.lib.frc1731.subsystem.converter;

import edu.wpi.first.units.measure.Angle;

public class AngularSubsystemConverter extends SubsystemConverter<Angle>{
    public AngularSubsystemConverter(double gearRatio) {
        super(gearRatio);
    }
    
    @Override
    public Angle toMotor(Angle mechanism) {
        return mechanism.div(gearRatioScalar);
    }
    
    @Override
    public Angle toMechanism(Angle motor) {
        return motor.times(gearRatioScalar);
    }
}