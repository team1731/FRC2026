package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerIOInputs {
    public double currentVelocityTop = 0.0;
    public double currentVelocityBottom = 0.0;  
    
    public double targetVelocityTop = 0.0;
    public double targetVelocityBottom = 0.0;

    public boolean atTargetVelocityTop = false;
    public boolean atTargetVelocityBottom = false;
}