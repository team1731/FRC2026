package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

@AutoLog
public class TurretIOInputs {
    public double targetDegrees = 0;
    public double currentDegrees = 0;
    public double minDegrees = 0;
    public double maxDegrees = 0;

    public boolean atTarget = false;

    public Translation2d target = new Translation2d();
    public Pose2d turretPose = new Pose2d();
}