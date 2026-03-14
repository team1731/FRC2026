package frc.lib.frc1731.hardware.motor;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.ctre.*;
import frc.lib.frc1731.hardware.motor.rev.*;
import frc.lib.frc1731.log.SmartLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

/**
 * Test class that treats a motor as a subsystem; useful for easy initial testing of motors and prototypes
 */
public class MotorIOTest extends SubsystemBase {
    protected MotorIO testMotor; // accesible to other test motors
    private double desiredOutput = 0d;
    private SmartLogger logger;
    private OutputType outputType = OutputType.kNone;

    private String dashboardTab;

    private enum OutputType {
        kPercentOutput,
        kVelocity,
        kPosition,
        kNone
    }

    public MotorIOTest(PortConfig config, Class<? extends MotorIO> cls) {
        if (cls.getSimpleName().equals(MotorIOSparkFlex.class.getSimpleName())) {
            this.testMotor = new MotorIOSparkFlex(config);
        } else if (cls.getSimpleName().equals(MotorIOSparkMax.class.getSimpleName())) {
            this.testMotor = new MotorIOSparkMax(config);
        } else if (cls.getSimpleName().equals(MotorIOTalonFX.class.getSimpleName())) {
            this.testMotor = new MotorIOTalonFX(config);
        } else if (cls.getSimpleName().equals(MotorIOTalonFXS.class.getSimpleName())) {
            this.testMotor = new MotorIOTalonFXS(config);
        }

        dashboardTab = "TestMotor/" + cls.getSimpleName() + "[" + config.kPort + "]";

        this.logger = new SmartLogger(dashboardTab);
        this.logger.log("Class Type", cls.getSimpleName());

        SmartDashboard.putNumber(dashboardTab + "/Tuneable Output", desiredOutput);
    }

    /**
     * Sets the maximum and minimum values that the motor can rotate to in rotations
     */
    public MotorIOTest setSoftLimits(double min, double max) {
        this.testMotor.setSoftLimits(min, max);
        return this;
    }

    /**
     * Adds an inverted follower motor to this test motor
     */
    public MotorIOTest withFollower(MotorIOTest other) {
        other.testMotor.follow(this.testMotor);
        return this;
    }

    /**
     * Adds PID gains to this test motor
     */
    public MotorIOTest withGains(PIDGains profile) {
        this.testMotor.withPIDGains(profile);
        return this;
    }

    /**
     * Get current position in rotations
     */
    public double getRotations() {
        return this.testMotor.getRotations();
    }

    /**
     * Get current velocity in RPM
     */
    public double getVelocityRPM() {
        return this.testMotor.getVelocityRPS() * 60d;
    }

    /**
     * Get the voltage applied to the motor
     */
    public double getAppliedVoltage() {
        return this.testMotor.getAppliedVoltage();
    }

    /**
     * Set the desired output of the motor to a percentage of maximum output
     */
    public Command setPercentOutput(double percent) {
        return run(() -> {
            this.desiredOutput = percent;
            this.testMotor.setPercentOutput(percent);
            this.outputType = OutputType.kPercentOutput;
        });
    }

    public Command setTuneablePercentOutput(double defaultValue) {
        return run(() -> {
            double percent = SmartDashboard.getNumber(dashboardTab + "/Tuneable Output", 0.0);
            this.desiredOutput = percent;
            this.testMotor.setPercentOutput(percent);
            this.outputType = OutputType.kPercentOutput;
        });
    }

    /**
     * Set the velocity of the motor in RPM
     */
    public Command setVelocityRPS(double velocityRPS) {
        return run(() -> {
            this.desiredOutput = velocityRPS;
            this.testMotor.setVelocityRPS(velocityRPS);
            this.outputType = OutputType.kVelocity;
        });
    }

    /**
     * Set the position of the motor in rotations
     */
    public Command setPosition(double rotations) {
        return run(() -> {
            this.desiredOutput = rotations;
            this.testMotor.setPosition(rotations);
            this.outputType = OutputType.kPosition;
        });
    }

    @Override
    public void periodic() {
        this.logger.log("Desired Output", desiredOutput);
        this.logger.log("Output Type", outputType);
    }
}
