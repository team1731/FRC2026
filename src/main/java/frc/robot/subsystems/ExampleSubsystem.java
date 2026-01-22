package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseSubsystem;

// Deprecated subsystem just to prevent accidental use
@Deprecated(forRemoval = false) 
public class ExampleSubsystem extends BaseSubsystem {
    private MotorIOTalonFX exampleMotor; // The running motor controller for this subsystem
    private double targetSetpoint = 0.0; // The updated target position for the motor to reach
    private double setpointTolerance = 0.1; // The acceptable error to be considered "at target"
    private boolean isAtSetpoint = false; // Whether the motor is currently at the target setpoint

    public ExampleSubsystem(boolean enabled) {
        super(enabled); // Calls the BaseSubsystem constructor and passes whether the subsystem is enabled or disabled
        this.exampleMotor = new MotorIOTalonFX(new PortConfig("exampleCANivore", 1, false)); // Creates a new TalonFX motor controller on CAN ID 1
        this.exampleMotor.withFollower(new MotorIOTalonFX(new PortConfig("exampleCANivore", 2, true))); // Creates a follower TalonFX on CAN ID 2 that is inverted from the master
        // Initializes the SysID routine with ramp rate, step rate, and timeout
        super.initSysId(1, 7, 10,
            voltage -> exampleMotor.setVoltage(voltage.in(Volts))
        );
    }

    @Override
    public void periodicTelemetry() {
        // Periodic logs that send useful information to both AdvantageScope and the USB if on an actual robot
        logger.log("Current Velocity", exampleMotor.getVelocityRPS());
        logger.log("Current Position", exampleMotor.getRotations());
        logger.log("Target Setpoint", targetSetpoint);
        logger.log("Is At Setpoint", isAtSetpoint);
    }

    /**
     * Checks if the motor is at the target setpoint within the defined tolerance
     */
    public boolean atSetpoint() {
        double currentVelocity = exampleMotor.getVelocityRPS();
        return Math.abs(currentVelocity - targetSetpoint) < setpointTolerance;
    }

    /**
     * Sets the target position for the motor
     */
    public Command setTargetPosition(double setpoint) {
        return this.run(() -> {
            this.targetSetpoint = setpoint;
            this.exampleMotor.setVelocityRPS(setpoint);
        });
    }
}