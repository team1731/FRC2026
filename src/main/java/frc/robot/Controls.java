package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Central location for driver/operator controller objects and named control bindings.
 *
 * <p>This class is useful when the team wants swappable control schemes. The active robot code
 * currently binds controls in {@link RobotContainer}, but this class provides a reusable pattern for
 * exposing triggers by robot intent.
 */
public class Controls {
    private final SendableChooser<ControlSet> controlChooser = new SendableChooser<>();

    private static final CommandPS5Controller driver = new CommandPS5Controller(RobotConstants.kDriverControllerPort);
    private static final CommandXboxController operator = new CommandXboxController(RobotConstants.kOperatorControllerPort);

    private ControlSet controlSet = ControlSet.kDefault;

    public Trigger resetSwerve = driver.options();
    public Trigger intake = driver.L2();
    public Trigger shoot = driver.R2();
    public Trigger pass = driver.R1();

    public Trigger shotFeedthrough = intake.and(shoot);
    public Trigger passFeedthrough = intake.and(pass);

    public Trigger warmup = operator.rightTrigger();
    public Trigger overrideHubShot = operator.a();
    public Trigger overrideTrenchShot = operator.x();
    public Trigger overrideTowerShot = operator.y();
    public Trigger overrideLobShot = operator.b();
    public Trigger shotOverride = overrideHubShot.or(overrideTrenchShot).or(overrideTowerShot).or(overrideLobShot);

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
    public Controls() {
        this.controlChooser.setDefaultOption("Default", ControlSet.kDefault);
        for (ControlSet set : ControlSet.values()) {
            if (!set.equals(ControlSet.kDefault)) { // Don't add the default value
                this.controlChooser.addOption(set.name(), set);
            }
        }

        // If there are any alternating sets of controls add them here
        switch (controlSet) {
            default:
        }
    }

    /**
     * Returns the shared driver controller.
     *
     * @return PS5 controller assigned to the driver port
     */
    public static CommandPS5Controller getDriver() {
        return Controls.driver;
    }

    /**
     * Returns the shared operator controller.
     *
     * @return Xbox controller assigned to the operator port
     */
    public static CommandXboxController getOperator() {
        return Controls.operator;
    }
}
