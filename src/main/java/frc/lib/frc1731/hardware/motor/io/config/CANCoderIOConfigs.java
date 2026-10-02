package frc.lib.frc1731.hardware.motor.io.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CANCoderIOConfigs {
    private CANcoderConfiguration cfg = new CANcoderConfiguration();

    public CANCoderIOConfigs(double offset, SensorDirectionValue direction) {
        this.cfg.MagnetSensor.MagnetOffset = offset;
        this.cfg.MagnetSensor.SensorDirection = direction;
    }

    public CANCoderIOConfigs withDiscontinuityPoint(double value) {
        this.cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = value;
        return this;
    }

    public CANcoderConfiguration build() {
        return this.cfg.clone();
    }
}