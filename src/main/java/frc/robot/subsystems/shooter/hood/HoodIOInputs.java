package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodIOInputs implements LoggableInputs {
    public double motorRotations = 0;
    public double targetRotations = 0;
    public boolean atTarget = false;

    @Override
    public void toLog(LogTable table) {
        table.put("motorRotations", motorRotations);
        table.put("targetRotations", targetRotations);
        table.put("atTarget", atTarget);
    }

    @Override
    public void fromLog(LogTable table) {
        motorRotations = table.get("motorRotations", motorRotations);
        targetRotations = table.get("targetRotations", targetRotations);
        atTarget = table.get("atTarget", atTarget);
    }
}