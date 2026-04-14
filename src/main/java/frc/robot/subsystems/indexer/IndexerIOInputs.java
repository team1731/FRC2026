package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerIOInputs {
    public double currentVelocityRight = 0.0;
    public double currentVelocityLeft = 0.0;  
    
    public double targetVelocityRight = 0.0;
    public double targetVelocityLeft = 0.0;

    public boolean atTargetVelocityLeft = false;
    public boolean atTargetVelocityRight = false;
}