package frc.robot;


import frc.lib.frc1731.hardware.motor.PortConfig;

public class Ports {
    public static final int kPivotCANcoderId = 15;

    public static final PortConfig kIntakeDeployCANCoderConfig = new PortConfig(RobotConstants.kMainCANBus, 15);

    public static final PortConfig kIntakeDeployConfig = new PortConfig(RobotConstants.kMainCANBus, 14);
    public static final PortConfig kIntakeRollerConfig = new PortConfig(RobotConstants.kSecondCANBus, 16); // flipped

    public static final PortConfig kIndexerFloorConfig = new PortConfig(RobotConstants.kSecondCANBus, 17); // flipped

    public static final PortConfig kBottomKickerMasterConfig = new PortConfig(RobotConstants.kSecondCANBus, 18); // flipped
    public static final PortConfig kTopKickerConfig = new PortConfig(RobotConstants.kSecondCANBus, 22); // flipped

    public static final PortConfig kHoodConfig = new PortConfig(RobotConstants.kSecondCANBus, 23);
    
    public static final PortConfig kLeftFlywheelTopMasterConfig = new PortConfig(RobotConstants.kSecondCANBus, 19);
    public static final PortConfig kRightFlywheelTopConfig = new PortConfig(RobotConstants.kSecondCANBus, 20); // flipped

    public static final PortConfig kLeftFlywheelBottomConfig = new PortConfig(RobotConstants.kSecondCANBus, 24);
    public static final PortConfig kRightFlywheelBottomConfig = new PortConfig(RobotConstants.kSecondCANBus, 25); // flipped

    public static final PortConfig kSqueezerConfig = new PortConfig(RobotConstants.kSecondCANBus, 26);
}