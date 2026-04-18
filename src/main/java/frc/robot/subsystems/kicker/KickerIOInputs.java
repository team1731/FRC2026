package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class KickerIOInputs {
    public double currentVelocity = 0.0;
    public double targetVelocity = 0.0;
    public boolean atTargetVelocity = false;
}