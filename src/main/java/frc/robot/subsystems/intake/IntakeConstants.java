package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class IntakeConstants {
    public static final PortConfig kRollerMotorConfig = new PortConfig(0);
    public static final PortConfig kPivotMotorConfig = new PortConfig(1);
    public static final PIDGains kPIDGains = new PIDGains().setP(1);
    
    public static final double kPivotIntakeRotations = 4.0; // rotations
    public static final double kPivotStowRotations = 0.0; // rotations
    public static final double kPositionTolerance = 0.2; // rotations

}
