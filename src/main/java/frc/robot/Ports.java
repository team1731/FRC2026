package frc.robot;

import frc.lib.frc1731.hardware.motor.PortConfig;

public class Ports {
    // All Ports not including swerve ports because that would be a little annoying
    public final static int kCANdleID = 35;

    public static final int kPivotCANcoderId = 15;

    public static final int kRightHoodCANCoderId = 21;
    public static final int kLeftHoodCANCoderId = 25;

    public static final int kClimbCANCoderId = 28;

    public static final int kLeftTurretCANCoderId = 29;
    public static final int kRightTurretCANCoderId = 30;

    public static final PortConfig kIntakePivotMotorConfig = new PortConfig(Constants.kRightCANBus, 14, false);
    public static final PortConfig kIntakeRollerMotorConfig = new PortConfig(Constants.kLeftCANBus, 16, true);

    public static final PortConfig kIndexerConfig = new PortConfig(Constants.kRightCANBus, 17, true);

    public static final PortConfig kRightTurretConfigs = new PortConfig(Constants.kRightCANBus, 18, false);
    public static final PortConfig kRightHoodConfig = new PortConfig(Constants.kRightCANBus, 19, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig(Constants.kRightCANBus, 20, true);

    public static final PortConfig kLeftTurretConfigs = new PortConfig(Constants.kLeftCANBus, 22, false);
    public static final PortConfig kLeftHoodConfig = new PortConfig(Constants.kLeftCANBus, 23, true);
    public static final PortConfig kLeftFlywheelConfig = new PortConfig(Constants.kLeftCANBus, 24, false);

    public static final PortConfig kClimbConfig = new PortConfig(Constants.kLeftCANBus, 27, false);
}