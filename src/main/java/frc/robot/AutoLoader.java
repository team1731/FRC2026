package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Owns the PathPlanner autonomous chooser and exposes the selected auto plus its start pose.
 */
public class AutoLoader {
    /** Shared autonomous loader instance used by {@link Robot}. */
    public static final AutoLoader kInstance = new AutoLoader();

    private SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final Command noAuto = Commands.none();
    
    private AutoLoader() {
        m_chooser = AutoBuilder.buildAutoChooser(RobotConstants.kAutoDefault);
        SmartDashboard.putData(RobotConstants.kAutoCodeKey, m_chooser);
    }

    /**
     * Returns the currently selected autonomous command.
     *
     * <p>The chooser's None option and an absent selection both safely do nothing.
     *
     * @return selected command, including the chooser's None option
     */
    public Command getSelected() {
        Command selected = m_chooser.getSelected();
        return selected != null ? selected : noAuto;
    }

    /**
     * Builds a new instance of the configured default auto.
     *
     * @return default PathPlanner auto
     */
    public PathPlannerAuto getDefault() {
        return new PathPlannerAuto(RobotConstants.kAutoDefault);
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
