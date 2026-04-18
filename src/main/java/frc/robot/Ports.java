package frc.robot;


import frc.lib.frc1731.hardware.motor.PortConfig;

public class Ports {
    // All Ports excluding swerve ports because that would be a little annoying
    public final static int kCANdleID = 41;

    public static final int kPivotCANcoderId = 15;

    public static final PortConfig kIntakePivotConfig = new PortConfig(RobotConstants.kMainCANBus, 14, false);
    public static final PortConfig kIntakeRollerConfig = new PortConfig(RobotConstants.kSecondCANBus, 16, true);

    public static final PortConfig kLeftIndexerConfig = new PortConfig(RobotConstants.kSecondCANBus, 40, false);
    public static final PortConfig kRightIndexerConfig = new PortConfig(RobotConstants.kMainCANBus, 17, true);

    public static final PortConfig kLeftKickerConfig = new PortConfig(RobotConstants.kSecondCANBus, 21, false);
    public static final PortConfig kRightKickerConfig = new PortConfig(RobotConstants.kMainCANBus, 21, true);

    public static final PortConfig kLeftHoodConfig = new PortConfig(RobotConstants.kSecondCANBus, 23, true);
    public static final PortConfig kRightHoodConfig = new PortConfig(RobotConstants.kMainCANBus, 19, false);
    
    public static final PortConfig kLeftFlywheelConfig = new PortConfig(RobotConstants.kSecondCANBus, 24, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig(RobotConstants.kMainCANBus, 20, true);
}