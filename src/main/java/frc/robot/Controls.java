package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.lib.frc1731.hardware.controller.SimplePS5Controller;
import frc.lib.frc1731.hardware.controller.SimpleXboxController;

/**
 * Central location for driver/operator controller objects and named control bindings.
 *
 * <p>This class is useful when the team wants swappable control schemes. The active robot code
 * currently binds controls in {@link RobotContainer}, but this class provides a reusable pattern for
 * exposing triggers by robot intent.
 */
public class Controls {
    private static final SendableChooser<ControlSet> controlChooser = new SendableChooser<>();

    private static final SimplePS5Controller driver = new SimplePS5Controller(RobotConstants.kDriverControllerPort);
    private static final SimpleXboxController operator = new SimpleXboxController(RobotConstants.kOperatorControllerPort);

    private static ControlSet controlSet = ControlSet.kDefault;

    public static Trigger resetSwerve = driver.rightOptions();
    public static Trigger snailDrive = driver.leftBumper();
    public static Trigger intake = driver.leftTrigger();
    public static Trigger shoot = driver.rightTrigger();
    public static Trigger pass = driver.rightBumper();
    public static Trigger collapseIntake = driver.dpadUp();

    public static Trigger shotFeedthrough = intake.and(shoot);
    public static Trigger passFeedthrough = intake.and(pass);

    public static Trigger warmup = operator.rightTrigger();
    public static Trigger overrideHubShot = operator.a();
    public static Trigger overrideTrenchShot = operator.x();
    public static Trigger overrideTowerShot = operator.y();
    public static Trigger overrideLobShot = operator.b();

    public Trigger raiseHopper = operator.rightBumper();
    public Trigger lowerHopper = operator.leftBumper();

    /** Available driver/operator control mapping presets. */
    public enum ControlSet {
        /** Default control map used by the robot unless another option is selected. */
        kDefault,
        // Add any additional sets of controls below for customizability
    }

    /**
     * Builds the control chooser and initializes the selected control mapping.
     */
    private Controls() {
        // this.controlChooser.setDefaultOption("Default", ControlSet.kDefault);
        // for (ControlSet set : ControlSet.values()) {
        //     if (!set.equals(ControlSet.kDefault)) { // Don't add the default value
        //         this.controlChooser.addOption(set.name(), set);
        //     }
        // }

        // // If there are any alternating sets of controls add them here
        // switch (controlSet) {
        //     default:
        // }
    }

    public static void build() {
        controlChooser.setDefaultOption("Default", ControlSet.kDefault);
        for (ControlSet set : ControlSet.values()) {
            if (!set.equals(ControlSet.kDefault)) { // Don't add the default value
                controlChooser.addOption(set.name(), set);
            }
        }

        // If there are any alternating sets of controls add them here
        switch (controlSet) {
            default:
        }

        SmartDashboard.putData("Choose Controls", controlChooser);
    }

    /**
     * Returns the shared driver controller.
     *
     * @return PS5 controller assigned to the driver port
     */
    public static SimplePS5Controller getDriver() {
        return Controls.driver;
    }

    /**
     * Returns the shared operator controller.
     *
     * @return Xbox controller assigned to the operator port
     */
    public static SimpleXboxController getOperator() {
        return Controls.operator;
    }
}
