package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;

/** Oscillates the position target between deployed and stowed once per second. */
public class JiggleToPosition extends Command {
    private static final double PERIOD_SECONDS = 1.0;
    private static final double MIN_POSITION = IntakeConstants.kPivotIntakeRotations;
    private static final double MAX_POSITION = IntakeConstants.kPivotStowRotations;
    private static final double CENTER_POSITION = (MIN_POSITION + MAX_POSITION) / 2.0;
    private static final double AMPLITUDE = (MAX_POSITION - MIN_POSITION) / 2.0;

    private final IntakePivotSubsystem intake;
    private final Timer timer = new Timer();
    private double initialPhase;

    public JiggleToPosition(IntakePivotSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        double position = MathUtil.clamp(intake.getPosition(), MIN_POSITION, MAX_POSITION);
        // Start at the measured position, initially moving toward stow, without a target jump.
        initialPhase = Math.acos(MathUtil.clamp(
            (CENTER_POSITION - position) / AMPLITUDE, -1.0, 1.0));
        intake.setPosition(position);
        timer.restart();
    }

    @Override
    public void execute() {
        double phase = initialPhase + 2.0 * Math.PI * timer.get() / PERIOD_SECONDS;
        intake.setPosition(CENTER_POSITION - AMPLITUDE * Math.cos(phase));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.setPosition(intake.getPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
