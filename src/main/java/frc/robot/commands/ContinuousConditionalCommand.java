package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Switches branches when the condition changes, remaining scheduled until interrupted. */
public class ContinuousConditionalCommand extends Command {
    private final Command whileTrue;
    private final Command whileFalse;
    private final BooleanSupplier condition;
    private Command selected;
    private boolean selectedValue;
    private boolean branchFinished;

    public ContinuousConditionalCommand(Command whileTrue, Command whileFalse, BooleanSupplier condition) {
        this.whileTrue = Objects.requireNonNull(whileTrue);
        this.whileFalse = Objects.requireNonNull(whileFalse);
        this.condition = Objects.requireNonNull(condition);
        CommandScheduler.getInstance().registerComposedCommands(whileTrue, whileFalse);
        addRequirements(whileTrue.getRequirements());
        addRequirements(whileFalse.getRequirements());
    }

    private void select(boolean value) {
        selectedValue = value;
        selected = value ? whileTrue : whileFalse;
        branchFinished = false;
        selected.initialize();
    }

    @Override
    public void initialize() { select(condition.getAsBoolean()); }

    @Override
    public void execute() {
        boolean value = condition.getAsBoolean();
        if (value != selectedValue) {
            if (!branchFinished) selected.end(true);
            select(value);
        }
        if (!branchFinished) {
            selected.execute();
            if (selected.isFinished()) {
                selected.end(false);
                branchFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!branchFinished) selected.end(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return whileTrue.runsWhenDisabled() && whileFalse.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return whileTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming
            && whileFalse.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming
            ? InterruptionBehavior.kCancelIncoming : InterruptionBehavior.kCancelSelf;
    }
}
