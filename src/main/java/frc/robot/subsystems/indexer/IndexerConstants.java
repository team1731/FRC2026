package frc.robot.subsystems.indexer;

import frc.lib.frc1731.PIDGains;

public class IndexerConstants {
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(1.0)
        .setV(0.12)
        .setS(0.15)
        ;
}
