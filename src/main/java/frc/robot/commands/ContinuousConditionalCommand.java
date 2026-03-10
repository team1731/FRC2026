package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class ContinuousConditionalCommand extends WrapperCommand {
    public ContinuousConditionalCommand(Command whileTrue, Command whileFalse, BooleanSupplier condition) {
        super(new DeferredCommand(() -> {
            if (condition.getAsBoolean()) return whileTrue;
            return whileFalse;
        }, whileTrue.getRequirements()));
    }
}