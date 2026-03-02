package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.SimpleCANdle;
import frc.robot.subsystems.BaseSubsystem;

public class LEDSubsystem extends BaseSubsystem {
    private SimpleCANdle candle;
    
    public LEDSubsystem(boolean enabled){
        super(enabled);
        if(!enabled) return;
        this.candle = new SimpleCANdle(1, "rio", 8);
    }

    public Command setFireCommand() {
        return this.run(() -> candle.setFire());
    }

    @Override
    public void periodicTelemetry() {}
}
