package frc.robot.subsystems.shooter.turret;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private Angle leftTargetAngle = Degrees.zero();
    private Angle rightTargetAngle = Degrees.zero();

    public TurretSubsystem(boolean enabled) {
        super(enabled);
        if (!enabled) return;
        leftMotor = new MotorIOTalonFX(kLeftPortConfigs);
        rightMotor = new MotorIOTalonFX(kRightPortConfigs);
        
        leftMotor.withPIDGains(kPositionGains);
        rightMotor.withPIDGains(kPositionGains);

        leftMotor.withStatorCurrentLimit(kCurrentLimit);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);

        leftMotor.setSoftLimits(-0.2453, 0.3806);
        leftMotor.setDynamicMotionMagicSpeeds(20, 20);
        leftMotor.withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(20)
        );

        leftMotor.withFeedbackConfigs(
            new FeedbackConfigs()
            .withFusedCANcoder(new CoreCANcoder(29, "Right CANivore"))
            .withRotorToSensorRatio(kRotorToSensor)
            .withSensorToMechanismRatio(kSensorToMech)
        );
        
        rightMotor.setSoftLimits(-0.7321, 0.1616);
        rightMotor.setDynamicMotionMagicSpeeds(40, 20);
        rightMotor.withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(1)
        );

        rightMotor.withFeedbackConfigs(
            new FeedbackConfigs()
            .withFusedCANcoder(new CoreCANcoder(30, "Left CANivore"))
            .withRotorToSensorRatio(kRotorToSensor)
            .withSensorToMechanismRatio(kSensorToMech)
        );

        Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        }));
    }

    public Angle getLeftTargetAngle() {
        return leftTargetAngle;
    }

    public Angle getRightTargetAngle() {
        return rightTargetAngle;
    }
    
    public Angle getLeftTurretAngle() {
        return kConverter.toMechanism(Rotations.of(leftMotor.getRotations()));
    }

    public Angle getRightTurretAngle() {
        return kConverter.toMechanism(Rotations.of(rightMotor.getRotations()));
    }

    public boolean atLeftTargetAngle() {
        return getLeftTurretAngle().isNear(getLeftTargetAngle(), kEpsilon);
    }

    public boolean atRightTargetAngle() {
        return getRightTurretAngle().isNear(getRightTargetAngle(), kEpsilon);
    }

    public boolean atBothTargetAngle() {
        return atLeftTargetAngle() && atRightTargetAngle();
    }

    public Pose3d getLeftTurretPose(Pose3d swervePose) {
        return swervePose.transformBy(kLeftTurretToRobot);
    }

    public Pose3d getRightTurretPose(Pose3d swervePose) {
        return swervePose.transformBy(kRightTurretToRobot);
    }

    public Transform3d getTransformToTarget(Pose3d swervePose, Pose3d targetPose) {
        return getLeftTurretPose(swervePose).minus(targetPose);
    }

    public double getLeftTurretPosition(Pose2d robotPose, Pose2d targetPose){
        Transform2d variance = targetPose.minus(robotPose);
        double turretAngle = Math.toDegrees(Math.atan(variance.getY()/variance.getX()));

        if (variance.getX() <= 0) {
            turretAngle += 180 * Math.signum(variance.getY());
        }

        turretAngle -= robotPose.getRotation().getDegrees();
        return turretAngle;
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Left/Motor Rotations", leftMotor.getRotations());
        logger.log("Left/Turret Degrees", getLeftTurretAngle().in(Degrees));
        logger.log("Left/Target Degrees", getLeftTargetAngle().in(Degrees));
        logger.log("Left/At Target", atLeftTargetAngle());

        logger.log("Right/Motor Rotations", rightMotor.getRotations());
        logger.log("Right/Turret Degrees", getRightTurretAngle().in(Degrees));
        logger.log("Right/Target Degrees", getRightTargetAngle().in(Degrees));
        logger.log("Right/At Target", atRightTargetAngle());
    }

    public Command setLeftTurretCommand(double targetDegrees) {
        return run(() -> {
            leftMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        });
    }

    public Command setRightTurretCommand(double targetDegrees) {
        return run(() -> {
            // rightMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
            this.rightTargetAngle = Degrees.of(targetDegrees);
            if (targetDegrees >= 266) {
                rightTargetAngle = rightTargetAngle.minus(Degrees.of(360));
            } else if (targetDegrees <= -128) {
                rightTargetAngle = rightTargetAngle.plus(Degrees.of(360));
            }

            SmartDashboard.putNumber("Right Turret Degrees", turretRotationsToDegrees(rightMotor.getRotations()));
            rightMotor.setPosition(degreesToTurretRotations(targetDegrees));
        });
    }

    public Command setRightTurretCommand(DoubleSupplier targetDegrees) {
        return run(() -> {
            // rightMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
            this.rightTargetAngle = Degrees.of(targetDegrees.getAsDouble());
            if (targetDegrees.getAsDouble() >= 266) {
                rightTargetAngle = rightTargetAngle.minus(Degrees.of(360));
            } else if (targetDegrees.getAsDouble() <= -128) {
                rightTargetAngle = rightTargetAngle.plus(Degrees.of(360));
            }

            SmartDashboard.putNumber("Right Turret Degrees", turretRotationsToDegrees(rightMotor.getRotations()));
            SmartDashboard.putNumber("Swerve yaw", targetDegrees.getAsDouble());
            rightMotor.setPosition(degreesToTurretRotations(rightTargetAngle.in(Degrees)));
        });
    }

    public Command setTurretCommand(double left, double right) {
        return run(() -> {
            this.leftTargetAngle = Degrees.of(left);
            this.rightTargetAngle = Degrees.of(right);

            leftMotor.setPosition(kConverter.toMotor(Degrees.of(left)).in(Rotations));
            rightMotor.setPosition(kConverter.toMotor(Degrees.of(right)).in(Rotations));
        });
    }

    // public Command setLeftRotations(double targetRotations) {
    //     return run(() -> {
    //         rightMotor.setPosition(targetRotations);
    //     });
    // }

    // public Command setRightRotations(double targetRotations) {
    //     return run(() -> {
    //         rightMotor.setPosition(targetRotations);
    //     });
    // }

    public double turretRotationsToDegrees(double rotations) {
        return (rotations + 0.44093) / 0.00226478;
    }

    public double degreesToTurretRotations(double degrees) {
        return 0.002678 * degrees - 0.44093;
    }


    public Command stopCommand() {
        return run(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        });
    }

    public Command driveManualCommand(double left, double right) {
        return run(() -> {
            // leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
            SmartDashboard.putNumber("Right Turret Degrees", turretRotationsToDegrees(rightMotor.getRotations()));
        });
    }

    public Command aimCommand(Supplier<Pose2d> swervePose, Supplier<Pose2d> targetPose) {
        return this.run(() -> {
            Transform2d leftTransform = new Transform2d(kLeftTurretToRobot.getX(), kLeftTurretToRobot.getY(), new Rotation2d());
            Transform2d rightTransform = new Transform2d(kRightTurretToRobot.getX(), kRightTurretToRobot.getY(), new Rotation2d());

            Transform2d leftVariance = targetPose.get().minus(swervePose.get().transformBy(leftTransform));
            Transform2d rightVariance = targetPose.get().minus(swervePose.get().transformBy(rightTransform));

            double leftAngle = Math.toDegrees(Math.atan(leftVariance.getY()/leftVariance.getX()));
            double rightAngle = Math.toDegrees(Math.atan(rightVariance.getY()/rightVariance.getX()));

            if (leftVariance.getX() <= 0) {
                leftAngle += 180 * Math.signum(leftVariance.getY());
            }

            if (rightVariance.getX() <= 0) {
                rightAngle += 180 * Math.signum(rightVariance.getY());
            }

            leftAngle -= swervePose.get().getRotation().getDegrees();
            rightAngle -= swervePose.get().getRotation().getDegrees();

            leftMotor.setPosition(kConverter.toMotor(Degrees.of(leftAngle)).in(Rotations));
            rightMotor.setPosition(kConverter.toMotor(Degrees.of(rightAngle)).in(Rotations));

            // Pose3d leftPose = new Pose3d(swervePose.get()).transformBy(kLeftTurretToRobot);
            // Pose3d rightPose = new Pose3d(swervePose.get()).transformBy(kRightTurretToRobot);

            // Transform3d leftTransformToTarget = new Pose3d(targetPose.get()).minus(leftPose);
            // Transform3d rightTransformToTarget = new Pose3d(targetPose.get()).minus(rightPose);

            // double leftTurretAngle = Math.toDegrees(Math.atan(leftTransformToTarget.getX() / leftTransformToTarget.getY())) - swervePose.get().getRotation().getDegrees();
            // double rightTurretAngle = Math.toDegrees(Math.atan(rightTransformToTarget.getX() / rightTransformToTarget.getY())) - swervePose.get().getRotation().getDegrees();

            // leftTurretAngle = Utils.clamp(leftTurretAngle, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));
            // rightTurretAngle = Utils.clamp(rightTurretAngle, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));

            // leftMotor.setPosition(kConverter.toMotor(Degrees.of(leftTurretAngle)).in(Rotations));
            // rightMotor.setPosition(kConverter.toMotor(Degrees.of(rightTurretAngle)).in(Rotations));
        });
    }
}