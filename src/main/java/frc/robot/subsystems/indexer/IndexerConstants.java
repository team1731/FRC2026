package frc.robot.subsystems.indexer;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class IndexerConstants {
    public static final PIDGains kPIDGains = new PIDGains().setP(1);
    public static final PortConfig kFollowerConfig = new PortConfig(1);
}
