package frc.robot.subsystems.shooter;

public class ShotEntry {
    public double hoodRotations;
    public double flywheelRPS;
    public double timeOfFlight;
    public double distance;

    public ShotEntry(double distance, double hood, double flywheel, double tof) {
        this.distance = distance;
        this.hoodRotations = hood;
        this.flywheelRPS = flywheel;
        this.timeOfFlight = tof;
    }
}