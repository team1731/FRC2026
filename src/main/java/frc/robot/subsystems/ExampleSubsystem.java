package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Robot;

// Deprecated subsystem just to prevent accidental use
@Deprecated(forRemoval = false) 
public class ExampleSubsystem extends BaseSubsystem {
    private MotorIOTalonFX exampleMotor; // The running motor controller for this subsystem
    private double targetSetpoint = 0.0; // The updated target position for the motor to reach
    private double setpointTolerance = 0.1; // The acceptable error to be considered "at target"
    private boolean isAtSetpoint = false; // Whether the motor is currently at the target setpoint

    public ExampleSubsystem(boolean enabled) {
        super(enabled); // Calls the BaseSubsystem constructor and passes whether the subsystem is enabled or disabled
    }

    @Override
    public void initializeHardware() {
        this.exampleMotor = new MotorIOTalonFX(new PortConfig("exampleCANivore", 1, false)); // Creates a new TalonFX motor controller on CAN ID 1
        this.exampleMotor.withFollower(new MotorIOTalonFX(new PortConfig("exampleCANivore", 2, true))); // Creates a follower TalonFX on CAN ID 2 that is inverted from the master
        
        // Initializes the SysID routine with ramp rate, step rate, and timeout
        super.initSysId(1d, 7d, 10d,
            volts -> setVoltage(volts), // This is where the command applies the voltage to the motors
            () -> getPosition(), // Supplies motor position to the logs for characterization
            () -> getVelocity(), // Supplies motor velocity to the logs for characterization
            () -> getVoltage() // Supplies motor voltage to the logs for characterization
        );
    }

    @Override
    public void periodicTelemetry() {
        // Periodic logs that send useful information to both AdvantageScope and the USB if on an actual robot
      //  logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
      //  logger.log("Current Position", getPosition().in(Rotations));
      //  logger.log("Target Setpoint", targetSetpoint);
      //  logger.log("Is At Setpoint", isAtSetpoint);
    }

    /**
     * Returns the current position of the motor
     */
    private Angle getPosition() {
        if (!isEnabled()) return Rotations.zero(); // If we are disabled, return 0
        if (Robot.isSimulation()) {
            // Return simulated position
        }
        return Rotations.of(exampleMotor.getRotations());
    }

    /**
     * Returns the current velocity of the motor
     */
    private AngularVelocity getVelocity() {
        if (!isEnabled()) return RotationsPerSecond.zero(); // If we are disabled, return 0
        if (Robot.isSimulation()) {
            // Return simulated velocity
        }
        return RotationsPerSecond.of(exampleMotor.getVelocityRPS());
    }

    /**
     * Returns the current voltage of the motor
     */
    private Voltage getVoltage() {
        if (!isEnabled()) return Volts.zero(); // If we are disabled, return 0
        if (Robot.isSimulation()) {
            // Return simulated voltage
        }
        return Volts.of(exampleMotor.getAppliedVoltage());
    }

    /**
     * Only used for sys id characterization since the tool uses voltage control to the motor
     */
    private void setVoltage(Voltage volts) {
        if (!isEnabled()) return; // If we are disabled, do nothing
        exampleMotor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation()) {
            // Apply the voltage to a simulation
        }
    }

    /**
     * Checks if the motor is at the target setpoint position within the defined tolerance
     */
    public boolean atSetpointPosition() {
        if (!isEnabled()) return true; // If we are disabled, always consider ourselves at position
        double currentPosition = getPosition().in(Rotations);
        return Math.abs(currentPosition - targetSetpoint) < setpointTolerance;
    }

    /**
     * Checks if the motor is at the target setpoint velocity within the defined tolerance
     */
    public boolean atSetpointVelocity() {
        if (!isEnabled()) return true; // If we are disabled, always consider ourselves at position
        double currentVelocity = getVelocity().in(RotationsPerSecond);
        return Math.abs(currentVelocity - targetSetpoint) < setpointTolerance;
    }

    /**
     * Sets the target position for the motor
     */
    public Command setPositionCommand(double setpoint) {
        return this.run(() -> {
            this.targetSetpoint = setpoint;
            this.exampleMotor.setPosition(setpoint);
            if (Robot.isSimulation()) {
                // Apply target position to the simulation
            }
        }).onlyIf(() -> isEnabled())
        .until(() -> atSetpointPosition())
        .withName("SetTargetPosition");
    }

    /**
     * Sets the target velocity for the motor
     */
    public Command setVelocityCommand(double setpoint) {
        return this.run(() -> {
            this.targetSetpoint = setpoint;
            this.exampleMotor.setVelocityRPS(setpoint);
            if (Robot.isSimulation()) {
                // Apply target velocity to the simulation
            }
        }).onlyIf(() -> isEnabled())
        .until(() -> atSetpointPosition())
        .withName("SetTargetVelocity");
    }
}