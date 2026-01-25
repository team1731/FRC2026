package frc.robot.subsystems.superstructure;

public class ShotProfile {
    public double turretDegrees, hoodDegrees, flywheelRPS;
    public boolean shouldShoot;

    public ShotProfile(double turret, double hood, double flywheel, boolean shoot) {
        this.turretDegrees = turret;
        this.hoodDegrees = hood;
        this.flywheelRPS = flywheel;
        this.shouldShoot = shoot;
    }
}