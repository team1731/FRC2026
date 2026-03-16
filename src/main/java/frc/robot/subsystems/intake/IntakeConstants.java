package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;

public class IntakeConstants {
    public static final PIDGains kPivotGains = new PIDGains()
        .setP(30)
        .setD(.5)
        .setS(.25)
        .setV(0.144)
        .setA(0.01);

    public static final PIDGains kRollerGains = new PIDGains().setP(0.01).setV(0.12);

    public static final double kRollerCurrentLimit = 40.0;
    public static final double kPivotCurrentLimit = 40.0;

    public static final double kPivotGearRatio = 48.0; // reduction

    public static final double kPivotIntakeRotations = -0.142;
    public static final double kPivotStowRotations = 0.0;
}