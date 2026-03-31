package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.SimpleCANdle;
import frc.robot.GameState;
import frc.robot.Ports;
import frc.robot.RobotConstants;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.leds.LEDConstants.*;

public class LEDSubsystem extends BaseSubsystem {
    private SimpleCANdle candle;
    
    public LEDSubsystem(boolean enabled){
        super(enabled);
    }

    @Override
    public void initializeHardware() {
        this.candle = new SimpleCANdle(Ports.kCANdleID, RobotConstants.kMainCANBus, kStripLength);
    }

    public Command setFire() {
        return this.run(() -> candle.setFire());
    }

    public Command flashAllianceShift() {
        return run(() -> {
            boolean activeShift = GameState.isMyHubActive();
            if (activeShift) {
                candle.setRainbow();
            } else {
                candle.setFire();
            }
        });
    }

    @Override
    public void periodicTelemetry() {}
}
