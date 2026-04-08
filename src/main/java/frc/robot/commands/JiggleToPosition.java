package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;

public class JiggleToPosition extends Command {
    private final IntakePivotSubsystem intake;
    private final Timer timer = new Timer();
    
    private final double START_POS = IntakeConstants.kPivotIntakeRotations;
    private final double END_POS = IntakeConstants.kPivotStowRotations;
    private final double DURATION = 3.0 / 2.0;
    private final double JIGGLE_AMPLITUDE = 0.1; // Distance of the "wiggle"

    public JiggleToPosition(IntakePivotSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        
        // 1. Calculate the moving "center" (Linear Interpolation)
        double progress = Math.min(time / DURATION, 1.0);
        double trendLine = START_POS + (progress * (END_POS - START_POS));
        
        // 2. Calculate the oscillation (The Sine Wave)
        // 2 * PI * 3 means 3 oscillations per second 
        double oscillation = Math.sin(time * Math.PI * 1.5) * JIGGLE_AMPLITUDE;
        
        // 3. Set the position
        intake.setPosition(trendLine - Math.abs(oscillation));
    }

    @Override
    public void end(boolean interrupted) {
        // Hold the final target position exactly at the end
        intake.setPosition(END_POS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
