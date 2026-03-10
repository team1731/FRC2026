package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class TurretSubsystemAI extends SubsystemBase {
    
    // Hardware & Requests
    private final TalonFX m_motor;
    private final CANcoder m_coder;
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);

    // Constants
    private final double m_forwardLimit; 
    private final double m_reverseLimit; 
    private final String m_name;
    private final Translation2d m_robotToTurret;
    private boolean inverted = false;

    // Targets
    public static final Translation2d BLUE_TARGET = new Translation2d(4.625594, 4.034536);
    public static final Translation2d RED_TARGET = new Translation2d(11.915394, 4.034536);

    // Suppliers
    private final Supplier<Pose2d> m_robotPoseSupplier;

    private double m_currentTargetDeg = 0.0;

    public TurretSubsystemAI(
        String name, int motorID, boolean inverted, int coderID, double offset, double discontinuity, 
        double reverseLimit, double forwardLimit, 
        double rotorToSensor, double sensorToMechanism,
        Translation2d robotToTurret,
        Supplier<Pose2d> poseSupplier,
        String canivore
    ) {
        this.m_name = name;
        this.m_motor = new TalonFX(motorID,canivore);
        this.m_coder = new CANcoder(coderID);
        this.inverted = inverted;
        this.m_reverseLimit = reverseLimit;
        this.m_forwardLimit = forwardLimit;
        this.m_robotToTurret = robotToTurret;
        this.m_robotPoseSupplier = poseSupplier;


        configureDevices(offset, discontinuity, rotorToSensor, sensorToMechanism);
    }

    private void configureDevices(double offset, double discontinuity, double rotorToSensor, double sensorToMechanism) {
        CANcoderConfiguration coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.MagnetOffset = offset;
        coderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuity;
        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_coder.getConfigurator().apply(coderConfig);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = m_coder.getDeviceID();
        motorConfig.Feedback.RotorToSensorRatio = rotorToSensor;

        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechanism;


        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_forwardLimit / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_reverseLimit / 360.0;
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

        m_motor.getConfigurator().apply(motorConfig);
        m_motor.setPosition(m_coder.getAbsolutePosition().waitForUpdate(0.2).getValueAsDouble());
    }

    @Override
    public void periodic() {
        double currentPosDeg = m_motor.getPosition().getValueAsDouble() * 360.0;

        //     if (m_shouldMove) {
        //         Pose2d robotPose = m_robotPoseSupplier.get();
        //         // ChassisSpeeds robotVel = m_robotVelocitySupplier.get(); // Must be field-relative speeds
                
        //         Translation2d actualGoal = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
        //             ? RED_TARGET : BLUE_TARGET;

        //         // 1. Calculate turret's current field position
        //         Translation2d turretFieldPos = robotPose.getTranslation().plus(
        //             m_robotToTurret.rotateBy(robotPose.getRotation())
        //         );

        //         // 2. Iterative Solver for Time of Flight (TOF)
        //         double timeOfFlight = turretFieldPos.getDistance(actualGoal) / NOTE_VELOCITY_MPS;
                
        //         // Second pass: Adjust TOF based on the virtual goal's distance
        //         for (int i = 0; i < 2; i++) {
        //             Translation2d virtualGoal = new Translation2d(
        //                 actualGoal.getX() - (0* timeOfFlight),
        //                 actualGoal.getY() - (0 * timeOfFlight)
        //             );
        //             timeOfFlight = turretFieldPos.getDistance(virtualGoal) / NOTE_VELOCITY_MPS;
        //         }

        //         // 3. Final Virtual Goal Calculation
        //         Translation2d finalVirtualGoal = new Translation2d(
        //             actualGoal.getX() - (0 * timeOfFlight),
        //             actualGoal.getY() - (0 * timeOfFlight)
        //         );

        //         // 4. Calculate Heading to the Virtual Goal
        //         double fieldTargetHeading = Math.toDegrees(Math.atan2(
        //             finalVirtualGoal.getY() - turretFieldPos.getY(),
        //             finalVirtualGoal.getX() - turretFieldPos.getX()
        //         ));

        //         m_currentTargetDeg = calculateBestTurretAngle(
        //             robotPose.getRotation().getDegrees(), 
        //             fieldTargetHeading, 
        //             currentPosDeg
        //         );
        //     } else {
        //         m_currentTargetDeg = 0.0; 
        //                     //  m_motor.setVoltage(0);
        //     }
        //     m_motor.setControl(m_mmRequest.withPosition(m_currentTargetDeg / 360.0));

        SmartDashboard.putNumber("Turret/" + m_name + " Angle", currentPosDeg);
        SmartDashboard.putBoolean("Turret/" + m_name + " Ready", isAtTarget(1.2));
        SmartDashboard.putNumber("Turret/Target", m_currentTargetDeg);
        SmartDashboard.putNumber("Turret/Reverse Limit", m_reverseLimit);
        SmartDashboard.putNumber("Turret/Forward Limit", m_forwardLimit);
        Translation2d pose = m_robotPoseSupplier.get().getTranslation().minus(m_robotToTurret.rotateBy(m_robotPoseSupplier.get().getRotation().plus(Rotation2d.k180deg)));
        Logger.recordOutput(m_name + "TurretPose", pose);
        Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;
        double distance = target.getDistance(pose);
        Logger.recordOutput(m_name + "TurretDistance", distance);
        // Logger.recordOutput(m_name + "TurretPose", m_robotPoseSupplier.get().getTranslation().minus(m_robotToTurret.rotateBy(m_robotPoseSupplier.get().getRotation())));
    }

    private double calculateBestTurretAngle(double robotHeading, double targetHeading, double current) {
        double desired = (targetHeading - robotHeading + 180) % 360;
        if (desired < 0) desired += 360;
        desired -= 180;

        double path1 = desired;
        double path2 = (desired > 0) ? (desired - 360) : (desired + 360);

        boolean reach1 = (path1 >= m_reverseLimit && path1 <= m_forwardLimit);
        boolean reach2 = (path2 >= m_reverseLimit && path2 <= m_forwardLimit);

        if (reach1 && reach2) {
            return (Math.abs(path1 - current) <= Math.abs(path2 - current)) ? path1 : path2;
        } else if (reach1) return path1;
        else if (reach2) return path2;

        return Math.max(m_reverseLimit, Math.min(m_forwardLimit, path1));
    }

    public boolean isAtTarget(double tolerance) {
        double currentPosDeg = m_motor.getPosition().getValueAsDouble() * 360.0;
        return Math.abs(m_currentTargetDeg - currentPosDeg) < tolerance;
    }

    // public void setTrackingEnabled(boolean enabled) {
    //     this.m_shouldMove = enabled;
    // }

    // public void setZero(boolean zero) {
    //     this.shouldZero = zero;
    // }

    public Command trackTargetCommand(Translation2d target) {
        return run(() -> {
            double currentPosDeg = m_motor.getPosition().getValueAsDouble() * 360.0;
            Pose2d robotPose = m_robotPoseSupplier.get();
            
            // Translation2d actualGoal = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
            //     ? RED_TARGET : BLUE_TARGET;

            // 1. Calculate turret's current field position
            Translation2d turretFieldPos = robotPose.getTranslation().plus(
                m_robotToTurret.rotateBy(robotPose.getRotation())
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
                target.getY() - turretFieldPos.getY(),
                target.getX() - turretFieldPos.getX()
            ));

            m_currentTargetDeg = calculateBestTurretAngle(
                robotPose.getRotation().getDegrees(), 
                fieldTargetHeading, 
                currentPosDeg
            );

            if (m_name.equals("Right")) return;
            m_motor.setControl(m_mmRequest.withPosition(m_currentTargetDeg / 360.0));
        });
    }

    public Command setZeroCommand() {
        return run(() -> {
            if (m_name.equals("Right")) return;
            m_motor.setControl(m_mmRequest.withPosition(0));
        });
    }

    public Command set180Command() {
        return run(() -> {
            if (m_name.equals("Right")) return;
            m_motor.setControl(m_mmRequest.withPosition(180));
        });
    }

    public Command setTargetCommand(double degrees) {
        return run(() -> {
            double output = degrees;
            if (degrees >= m_forwardLimit) {
                output -= 360;
            } else if (degrees <= m_reverseLimit) {
                output += 360;
            }
            if (m_name.equals("Right")) return;
            m_motor.setControl(m_mmRequest.withPosition(output));
        });
    }

    public Command trackHubCommand() {
        return trackTargetCommand(Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET);
    }

    // public Command setTrackingCommand(boolean enabled) {
    //     return runOnce(() -> {
    //         this.setTrackingEnabled(enabled);
    //     });
    // }

    // public Command setZeroCommand(boolean enabled) {
    //     return run(() -> {
    //         this.setZero(enabled);
    //     });
    // }
}