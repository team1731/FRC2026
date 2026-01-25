package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.VelocitySubsystem;

public class FlywheelSubsystem extends VelocitySubsystem<MotorIOTalonFX> {
    private double setPointVelocity = 0.0;
    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        SmartDashboard.putNumber("SetPointVelocity", setPointVelocity);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(new PortConfig(0));
        motor.withPIDGains(new PIDGains().setP(1.0/3000));
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", motor.getVelocityRPS());
        logger.log("Target Velocity", super.getTargetVelocityRPS());

    }
    public Command shootCommand() {
        return run(() -> {
            setPointVelocity = SmartDashboard.getNumber("SetPointVelocity", 0.0);
            motor.setPercentOutput(setPointVelocity);
        }); 
    }
    public Command stopCommand() {
        return run(() -> {
            setPointVelocity = 0.0;
            motor.setPercentOutput(0.0);
        });
    }
    
}
