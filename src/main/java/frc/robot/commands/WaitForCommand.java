package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.*;

/**
 * Command that waits for a condition to be met before proceeding
 */
public class WaitForCommand extends WrapperCommand {
    public WaitForCommand(Command command, BooleanSupplier condition) {
        super(new WaitCommand(Double.POSITIVE_INFINITY).until(condition).andThen(command));
    }
}