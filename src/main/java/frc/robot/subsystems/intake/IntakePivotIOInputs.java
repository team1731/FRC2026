package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakePivotIOInputs {
    public double targetPosition = 0.0;
    public double currentPosition = 0.0;
    public boolean atTargetPosition = false;
}