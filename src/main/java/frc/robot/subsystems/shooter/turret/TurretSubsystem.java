package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.field.FieldPositions;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private CANcoder cancoder;
    private Translation2d robotToTurret;
    private Supplier<Pose2d> swervePoseSupplier;

    private double targetDegrees = 0;
    private double minDegrees, maxDegrees;

    private Translation2d targetTranslation;

    private String name;

    public TurretSubsystem(TurretConfiguration config, Supplier<Pose2d> swervePoseSupplier, boolean enabled) {
        super(config.name(), config, enabled);
        this.swervePoseSupplier = swervePoseSupplier;
        this.name = config.name();
    }

    @Override
    protected void initializeHardware() {
        TurretConfiguration turretConfig = (TurretConfiguration)config.get();
        motor = new MotorIOTalonFX(turretConfig.motorConfigs());
        cancoder = new CANcoder(turretConfig.cancoderID(), turretConfig.motorConfigs().kBus);

        CANcoderConfiguration coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.MagnetOffset = turretConfig.cancoderConfigs().MagnetSensor.MagnetOffset;
        coderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = turretConfig.cancoderConfigs().MagnetSensor.AbsoluteSensorDiscontinuityPoint;
        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(coderConfig);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        motorConfig.Feedback.RotorToSensorRatio = turretConfig.feedbackConfigs().RotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = turretConfig.feedbackConfigs().SensorToMechanismRatio;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = turretConfig.motorConfigs().kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretConfig.maxDegrees() / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = turretConfig.minDegrees() / 360.0;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 30d;

        motorConfig.Slot0.kP = 60.0; 
        motorConfig.Slot0.kS = 0.2;
        motorConfig.Slot0.kA = 0.01;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 0.5;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
        motorConfig.MotionMagic.MotionMagicAcceleration = 4;

        motor.getMotor().getConfigurator().apply(motorConfig);
        motor.getMotor().setPosition(cancoder.getAbsolutePosition().waitForUpdate(0.2).getValueAsDouble());

        this.robotToTurret = turretConfig.robotToTurret().toTranslation2d();
        this.minDegrees = turretConfig.minDegrees();
        this.maxDegrees = turretConfig.maxDegrees();
    }

    public Pose2d getTurretPose() {
        if (!isEnabled()) return new Pose2d();
        Translation2d swerve = swervePoseSupplier.get().getTranslation();
        return new Pose2d(swerve.plus(robotToTurret.rotateBy(swervePoseSupplier.get().getRotation())), new Rotation2d());
    }

    public boolean atTarget() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations() * 360.0, targetDegrees, kEpsilon);
    }

    public boolean atTarget(double epsilon) {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations() * 360.0, targetDegrees, epsilon);
    }

    public double getTarget() {
        if (!isEnabled()) return 0;
        return targetDegrees;
    }

    public double getDegrees() {
        if (!isEnabled()) return 0;
        return motor.getRotations() * 360;
    }

    private double calculateBestTurretAngle(double robotHeading, double targetHeading, double current) {
        // double desired = (targetHeading - robotHeading) % 360;
        // if (desired > maxDegrees) desired -= 360;
        // if (desired < minDegrees) desired += 360;
        // return Math.max(minDegrees, Math.min(maxDegrees, desired));
        double desired = (targetHeading - robotHeading + 180) % 360;
        if (desired < 0) desired += 360;
        desired -= 180;

        double path1 = desired;
        double path2 = (desired > 0) ? (desired - 360) : (desired + 360);

        boolean reach1 = (path1 >= minDegrees && path1 <= maxDegrees);
        boolean reach2 = (path2 >= minDegrees && path2 <= maxDegrees);

        if (reach1 && reach2) {
            return (Math.abs(path1 - current) <= Math.abs(path2 - current)) ? path1 : path2;
        } else if (reach1) return path1;
        else if (reach2) return path2;

        return Math.max(minDegrees, Math.min(maxDegrees, path1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + " Turret Distance", FieldPositions.kHub.get().minus(getTurretPose().getTranslation()).getNorm());
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Pose", getTurretPose());
        logger.log("Current Degrees", motor.getRotations() * 360.0);
        logger.log("Target Degrees", targetDegrees);
        logger.log("At Target", atTarget());
        logger.log("Min Degrees", minDegrees);
        logger.log("Max Degrees", maxDegrees);

        if (targetTranslation != null) logger.log("TargetDistance", targetTranslation.minus(getTurretPose().getTranslation()).getNorm());
    }

    public Command trackHub() {
        return track(FieldPositions.kHub.get());
    }

    public Command track(Supplier<Translation2d> target) {
        return this.run(() -> {
            // this.targetTranslation = target.get();
            // Translation2d turretToTarget = target.get().minus(getTurretPose().getTranslation());
            // return Math.toDegrees(Math.atan(turretToTarget.getY() / turretToTarget.getX()));
            double currentPosDeg = motor.getRotations() * 360.0;
            Pose2d robotPose = swervePoseSupplier.get();
            
            // Translation2d actualGoal = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
            //     ? RED_TARGET : BLUE_TARGET;

            // 1. Calculate turret's current field position
            Translation2d turretFieldPos = robotPose.getTranslation().plus(
                robotToTurret.rotateBy(robotPose.getRotation())
            );

            // 2. Iterative Solver for Time of Flight (TOF)
            // double timeOfFlight = turretFieldPos.getDistance(actualGoal) / NOTE_VELOCITY_MPS;
            
            // // Second pass: Adjust TOF based on the virtual goal's distance
            // for (int i = 0; i < 2; i++) {
            //     Translation2d virtualGoal = new Translation2d(
            //         actualGoal.getX() - (0* timeOfFlight),
            //         actualGoal.getY() - (0 * timeOfFlight)
            //     );
            //     timeOfFlight = turretFieldPos.getDistance(virtualGoal) / NOTE_VELOCITY_MPS;
            // }

            // // 3. Final Virtual Goal Calculation
            // Translation2d finalVirtualGoal = new Translation2d(
            //     actualGoal.getX() - (0 * timeOfFlight),
            //     actualGoal.getY() - (0 * timeOfFlight)
            // );

            // 4. Calculate Heading to the Virtual Goal
            double fieldTargetHeading = Math.toDegrees(Math.atan2(
                target.get().getY() - turretFieldPos.getY(),
                target.get().getX() - turretFieldPos.getX()
            ));

            double output = calculateBestTurretAngle(
                robotPose.getRotation().getDegrees(), 
                fieldTargetHeading, 
                currentPosDeg
            );

            this.targetDegrees = output;

            motor.setPosition(output / 360.0);
        });
    }

    public Command track(Translation2d target) {
        return this.track(() -> target);
    }

    public Command setDegrees(DoubleSupplier degrees) {
        return run(() -> {
            double output = degrees.getAsDouble();
            if (degrees.getAsDouble() >= maxDegrees) {
                output -= 360;
            } else if (degrees.getAsDouble() <= minDegrees) {
                output += 360;
            }
            this.targetDegrees = output;
            motor.setPosition(output / 360.0);
        });
    }

    public Command setDegrees(double degrees) {
        return this.setDegrees(() -> degrees);
    }
}