package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;
public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private double leftTargetDegrees = 0;
    private double rightTargetDegrees = 0;

    private double leftRotations = 0;
    private double rightRotations = 0;

    private double leftDegrees = 0;
    private double rightDegrees = 0;

    public TurretSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        leftMotor = new MotorIOTalonFX(Ports.kLeftTurretConfigs);
        leftMotor.withPIDGains(kPositionGains);
        leftMotor.setSoftLimits(kLeftRotationsRange[0], kLeftRotationsRange[1]);
        leftMotor.withStatorCurrentLimit(kCurrentLimit);
        leftMotor.withMotionMagicConfigs(kMotionMagicConfigs);
        leftMotor.withFeedbackConfigs(kLeftFeedbackConfigs);

        rightMotor = new MotorIOTalonFX(Ports.kRightTurretConfigs);
        rightMotor.withPIDGains(kPositionGains);
        rightMotor.setSoftLimits(kRightRotationsRange[0], kRightRotationsRange[1]);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);
        rightMotor.withMotionMagicConfigs(kMotionMagicConfigs);
        rightMotor.withFeedbackConfigs(kRightFeedbackConfigs);
    }

    public double rightTurretRotationsToDegrees(double rotations) {
        return (rotations + 0.44093) / 0.00226478;
    }

    public double rightDegreesToTurretRotations(double degrees) {
        return 0.002678 * degrees - 0.44093;
    }

    public double leftTurretRotationsToDegrees(double rotations) {
        return (rotations + 0.011) / 0.00226478;
    }

    public double leftDegreesToTurretRotations(double degrees) {
        return 0.002678 * degrees - 0.011;
    }

    public boolean atLeftTargetRotations() {
        return Utils.isWithin(leftDegrees, leftTargetDegrees, kEpsilon);
    }

    public boolean atRightTargetRotations() {
        return Utils.isWithin(rightDegrees, rightTargetDegrees, kEpsilon);
    }

    public boolean atBothTargetRotations() {
        return atLeftTargetRotations() && atRightTargetRotations();
    }

    @Override
    public void periodicTelemetry() {
        leftRotations = leftMotor.getRotations();
        rightRotations = rightMotor.getRotations();

        leftDegrees = leftTurretRotationsToDegrees(leftRotations);
        rightDegrees = leftTurretRotationsToDegrees(rightRotations);

        logger.log("Left/Rotations", leftRotations);
        logger.log("Left/Degrees", leftDegrees);
        logger.log("Left/Target Degrees", leftTargetDegrees);
        logger.log("Left/At Target", atLeftTargetRotations());

        logger.log("Right/Rotations", rightRotations);
        logger.log("Right/Degrees", rightDegrees);
        logger.log("Right/Target Degrees", rightTargetDegrees);
        logger.log("Right/At Target", atRightTargetRotations());

        logger.log("At Both Target", atBothTargetRotations());
    }

    public Command setLeftDegrees(DoubleSupplier degrees) {
        return run(() -> {
            leftTargetDegrees = degrees.getAsDouble() % 360;
            if (leftTargetDegrees >= 266) {
                leftTargetDegrees -= 360;
            } else if (leftTargetDegrees <= -128) {
                leftTargetDegrees += 360;
            }

            leftMotor.setPosition(leftDegreesToTurretRotations(leftTargetDegrees));
        });
    }

    public Command setLeftDegrees(double degrees) {
        return setLeftDegrees(() -> degrees);
    }

    public Command setRightDegrees(DoubleSupplier degrees) {
        return run(() -> {
            rightTargetDegrees = degrees.getAsDouble() % 360;
            if (rightTargetDegrees >= 266) {
                rightTargetDegrees -= 360;
            } else if (rightTargetDegrees <= -128) {
                rightTargetDegrees += 360;
            }

            rightMotor.setPosition(rightDegreesToTurretRotations(rightTargetDegrees));
        });
    }

    public Command setRightDegrees(double degrees) {
        return setRightDegrees(() -> degrees);
    }

    public Command setDegrees(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            leftTargetDegrees = left.getAsDouble() % 360;
            rightTargetDegrees = right.getAsDouble() % 360;

            if (leftTargetDegrees >= 266) {
                leftTargetDegrees -= 360;
            } else if (leftTargetDegrees <= -128) {
                leftTargetDegrees += 360;
            }

            if (rightTargetDegrees >= 266) {
                rightTargetDegrees -= 360;
            } else if (rightTargetDegrees <= -128) {
                rightTargetDegrees += 360;
            }

            leftMotor.setPosition(leftDegreesToTurretRotations(leftTargetDegrees));
            rightMotor.setPosition(rightDegreesToTurretRotations(rightTargetDegrees));
        });
    }

    public Command setDegrees(double left, double right) {
        return setDegrees(() -> left, () -> right);
    }

    public Command aim(Supplier<Pose2d> swervePose, Supplier<Pose2d> targetPose) {
        return Commands.defer(() -> {
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

            return setDegrees(leftAngle, rightAngle);
        }, Set.of(this));
    }

    public Command lockStraight() {
        return setDegrees(() -> 0, () -> 0);
    }

    public Command driveManual(double left, double right) {
        return run(() -> {
            leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }

    public Command stopCommand() {
        return driveManual(0d, 0d).withTimeout(0);
    }
}