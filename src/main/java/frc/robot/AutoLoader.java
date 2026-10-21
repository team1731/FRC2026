package frc.robot;

import java.util.Optional;
import java.util.Locale;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Owns the PathPlanner autonomous chooser and exposes the selected auto plus its start pose.
 */
public class AutoLoader {
    private final SendableChooser<Command> chooser;
    private final PathPlannerAuto defaultAuto;
    
    public AutoLoader() {
        defaultAuto = new PathPlannerAuto(RobotConstants.kAutoDefault);
        chooser = new SendableChooser<>();
        chooser.setDefaultOption(displayName(RobotConstants.kAutoDefault), defaultAuto);
        AutoBuilder.getAllAutoNames().stream()
            .filter(name -> name.toLowerCase(Locale.ROOT).startsWith("comp_"))
            .filter(name -> !name.equals(RobotConstants.kAutoDefault))
            .sorted()
            .forEach(name -> chooser.addOption(displayName(name), new PathPlannerAuto(name)));
        chooser.onChange(cmd -> System.out.println("@@@@@@@@@ Selected New Auto: " + cmd.getName()));
        SmartDashboard.putData(RobotConstants.kAutoCodeKey, chooser);
    }

    /** Removes the blue-origin prefix from dashboard labels only. */
    private static String displayName(String name) {
        return name.toLowerCase(Locale.ROOT).startsWith("comp_") ? name.substring(5) : name;
    }

    /**
     * Returns the currently selected autonomous command.
     *
     * <p>If the selected auto does not exist, return the default instead.
     *
     * @return selected blue-origin auto, or the configured default
     */
    public Command getSelected() {
        Command selected = chooser.getSelected();
        return selected != null ? selected : getDefault();
    }

    /**
     * Returns the cached configured default auto.
     *
     * @return default PathPlanner auto
     */
    public PathPlannerAuto getDefault() {
        return defaultAuto;
    }

    /**
     * Starting position of the robot relative to the current alliance and selected autonomous mode
     */
    public Optional<Pose2d> getStartPose() {
        if (!(getSelected() instanceof PathPlannerAuto selected)) return Optional.empty();
        return Optional.ofNullable(selected.getStartingPose())
            .map(pose -> Robot.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
    }
}
