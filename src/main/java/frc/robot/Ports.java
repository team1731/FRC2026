package frc.robot;


import frc.lib.frc1731.hardware.motor.PortConfig;

public class Ports {
    // All Ports excluding swerve ports because that would be a little annoying
    public final static int kCANdleID = 41;

    public static final int kPivotCANcoderId = 15;

    public static final PortConfig kIntakePivotConfig = new PortConfig(RobotConstants.kMainCANBus, 14, false);
    public static final PortConfig kIntakeRollerConfig = new PortConfig(RobotConstants.kSecondCANBus, 16, true);

    public static final PortConfig kIndexerFloorConfig = new PortConfig(RobotConstants.kSecondCANBus, 17, true);

    public static final PortConfig kBottomtKickerConfig = new PortConfig(RobotConstants.kSecondCANBus, 18, true);
    public static final PortConfig kToptKickerConfig = new PortConfig(RobotConstants.kSecondCANBus, 22, true);

    public static final PortConfig kHoodConfig = new PortConfig(RobotConstants.kSecondCANBus, 23, false);
    
    public static final PortConfig kLeftFlywheelTopConfig = new PortConfig(RobotConstants.kSecondCANBus, 19, false);
    public static final PortConfig kRightFlywheelTopConfig = new PortConfig(RobotConstants.kSecondCANBus, 20, true);

    public static final PortConfig kLeftFlywheelBottomConfig = new PortConfig(RobotConstants.kSecondCANBus, 24, false);
    public static final PortConfig kRightFlywheelBottomConfig = new PortConfig(RobotConstants.kSecondCANBus, 25, true);

    public static final PortConfig kSqueezerConfig = new PortConfig(RobotConstants.kSecondCANBus, 26, false);
}