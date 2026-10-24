package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;

/** Read-only pit telemetry. DataLogManager records these NetworkTables values. */
public final class PitTelemetry {
    private final PowerDistribution power = new PowerDistribution();
    private double nextUpdate;
    private double minimumVoltage = Double.POSITIVE_INFINITY;
    private double peakCurrent;
    private int brownoutCount;
    private boolean wasBrownedOut;

    public void update() {
        double voltage = RobotController.getBatteryVoltage();
        double current = power.getTotalCurrent();
        boolean brownedOut = RobotController.isBrownedOut();
        if (brownedOut && !wasBrownedOut) brownoutCount++;
        wasBrownedOut = brownedOut;
        minimumVoltage = Math.min(minimumVoltage, voltage);
        peakCurrent = Math.max(peakCurrent, current);
        double now = Timer.getFPGATimestamp();
        if (now < nextUpdate) return;
        nextUpdate = now + 0.1;

        SmartDashboard.putNumber("Pit/Power/BatteryVoltage", voltage);
        SmartDashboard.putNumber("Pit/Power/PDVoltage", power.getVoltage());
        SmartDashboard.putNumber("Pit/Power/TotalSupplyAmps", current);
        SmartDashboard.putNumber("Pit/Power/MinimumVoltageSinceBoot", minimumVoltage);
        SmartDashboard.putNumber("Pit/Power/PeakSupplyAmpsSinceBoot", peakCurrent);
        SmartDashboard.putBoolean("Pit/Power/BrownedOut", brownedOut);
        SmartDashboard.putNumber("Pit/Power/BrownoutCountSinceBoot", brownoutCount);
        for (int channel = 0; channel < power.getNumChannels(); channel++) {
            SmartDashboard.putNumber("Pit/Power/Channel" + channel + "Amps", power.getCurrent(channel));
        }
        publish("Indexer", Robot.indexer);
        publish("Kicker", Robot.kicker);
        publish("Flywheel", Robot.flywheel);
    }

    private void publish(String name, BaseVelocitySubsystem<MotorIOTalonFX> subsystem) {
        String key = "Pit/" + name + "/";
        MotorIOTalonFX motor = subsystem.getMotor();
        double actual = subsystem.getVelocity().in(RotationsPerSecond);
        double requested = motor.getRequestType().isVelocityRequest()
            ? subsystem.getSetpoint().in(RotationsPerSecond) : 0.0;
        SmartDashboard.putNumber(key + "RequestedMechanismRPS", requested);
        SmartDashboard.putBoolean(key + "VelocityControlActive", motor.getRequestType().isVelocityRequest());
        SmartDashboard.putNumber(key + "ActualMechanismRPS", actual);
        SmartDashboard.putNumber(key + "ErrorRPS", requested - actual);
        SmartDashboard.putNumber(key + "LeaderVoltage", motor.getAppliedVoltage());
        SmartDashboard.putNumber(key + "LeaderDutyCycle", motor.getAppliedDutyCycle());
        SmartDashboard.putNumber(key + "LeaderSupplyAmps", motor.getSupplyCurrent());
        SmartDashboard.putNumber(key + "LeaderStatorAmps", motor.getStatorCurrent());
        double totalSupply = motor.getSupplyCurrent();
        for (var follower : motor.getFollowerMotors()) {
            double supply = follower.getSupplyCurrent().getValueAsDouble();
            String followerKey = key + "Follower" + follower.getDeviceID() + "/";
            SmartDashboard.putNumber(followerKey + "SupplyAmps", supply);
            SmartDashboard.putNumber(followerKey + "StatorAmps", follower.getStatorCurrent().getValueAsDouble());
            totalSupply += supply;
        }
        SmartDashboard.putNumber(key + "TotalSupplyAmps", totalSupply);
    }
}
