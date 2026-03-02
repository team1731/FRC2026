package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class IntakeConstants {
    public static final PortConfig kRollerMotorConfig = new PortConfig("Right CANivore", 16, true);
    public static final PortConfig kPivotMotorConfig = new PortConfig("Left CANivore", 14);
    public static final PIDGains kPivotGains = new PIDGains()
        .setP(20d)
        .setD(0.1)
        .setS(0.5)
        .setV(0.25)
        .setA(0.1);

    public static final PIDGains kPIDGains = new PIDGains().setP(1);
    
    // public static final double kPivotIntakeRotations = 3.5; // rotations
    // public static final double kPivotStowRotations = 0.0; // rotations
    // public static final double kPositionTolerance = 0.2; // rotations

    public static final double kPivotIntakeRotations = -0.091;
    public static final double kPivotStowRotations = 0.038;

    // public static final double kMaxRotations = 3.726;

}
