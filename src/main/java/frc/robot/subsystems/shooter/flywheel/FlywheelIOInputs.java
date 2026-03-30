package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FlywheelIOInputs {
    public double targetVelocity = 0.0;
    public double currentVelocity = 0.0;
    public boolean atTargetVelocity = false;
}