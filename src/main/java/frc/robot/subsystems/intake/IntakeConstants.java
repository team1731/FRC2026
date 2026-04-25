package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;

public class IntakeConstants {
    public static final PIDGains kPivotGains = new PIDGains()
        .setP(60)
        .setD(.5)
        // .setS(.25)
        .setV(5.76)
        .setA(0.48);

    public static final PIDGains kRollerGains = new PIDGains().setP(0.25).setV(0.12);

    public static final double kRollerCurrentLimit = 120;
    public static final double kPivotCurrentLimit = 60.0;

    public static final double kPivotGearRatio = 48.0; // reduction

    public static final double kPivotIntakeRotations = -0.13916;
    public static final double kPivotStowRotations = 0;
    public static final double kPivotEpsilon = 0.01;
}