package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class IntakeConstants {
    public static final PortConfig kMotorConfig = new PortConfig(0);
    public static final PortConfig kPivotMotorConfig = new PortConfig(1);
    public static final PIDGains kPIDGains = new PIDGains().setP(1);
    
}
