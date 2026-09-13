package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;

/** Collapses the intake and attached hopper over a fixed time without oscillating. */
public class JiggleToPosition extends Command {
    private final IntakePivotSubsystem intake;
    private final Timer timer = new Timer();
    private static final double DURATION_SECONDS = 1.5;
    private double startPosition;

    public JiggleToPosition(IntakePivotSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        startPosition = MathUtil.clamp(intake.getPosition(),
            IntakeConstants.kPivotIntakeRotations, IntakeConstants.kPivotStowRotations);
        intake.setPosition(startPosition);
        timer.restart();
    }

    @Override
    public void execute() {
        double progress = MathUtil.clamp(timer.get() / DURATION_SECONDS, 0.0, 1.0);
        // Smoothstep starts and ends the commanded trajectory at zero velocity.
        double blend = progress * progress * (3.0 - 2.0 * progress);
        intake.setPosition(startPosition
            + (IntakeConstants.kPivotStowRotations - startPosition) * blend);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.setPosition(interrupted ? intake.getPosition() : IntakeConstants.kPivotStowRotations);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DURATION_SECONDS);
    }
}
