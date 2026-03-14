package frc.robot.subsystems.feeder;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class FeederConstants {
    public static final PortConfig kMotorConfig = new PortConfig("Left CANivore", 17, true);
    public static final PIDGains kPIDGains = new PIDGains().setP(1);
    public static final PortConfig kFollowerConfig = new PortConfig(1);
}
