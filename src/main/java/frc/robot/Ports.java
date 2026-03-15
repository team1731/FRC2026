package frc.robot;


import frc.lib.frc1731.hardware.motor.PortConfig;

public class Ports {
    // All Ports not including swerve ports because that would be a little annoying
    public final static int kCANdleID = 35;

    public static final int kPivotCANcoderId = 15;

    // public static final int kRightHoodCANCoderId = 21;
    // public static final int kLeftHoodCANCoderId = 25;

    public static final int kLeftTurretCANCoderId = 29;
    public static final int kRightTurretCANCoderId = 30;

    public static final PortConfig kIntakePivotConfig = new PortConfig(RobotConstants.kMainCANBus, 14, false);
    public static final PortConfig kIntakeRollerConfig = new PortConfig(RobotConstants.kSecondCANBus, 16, true);

    public static final PortConfig kIndexerConfig = new PortConfig(RobotConstants.kMainCANBus, 17, true);

    public static final PortConfig kRightTurretConfigs = new PortConfig(RobotConstants.kMainCANBus, 18, false);
    public static final PortConfig kRightHoodConfig = new PortConfig(RobotConstants.kMainCANBus, 19, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig(RobotConstants.kMainCANBus, 20, true);

    public static final PortConfig kLeftTurretConfigs = new PortConfig(RobotConstants.kSecondCANBus, 22, true);
    public static final PortConfig kLeftHoodConfig = new PortConfig(RobotConstants.kSecondCANBus, 23, true);
    public static final PortConfig kLeftFlywheelConfig = new PortConfig(RobotConstants.kSecondCANBus, 24, false);
}