package frc.robot.subsystems.shooter.turret;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private Angle leftTargetAngle = Degrees.zero();
    private Angle rightTargetAngle = Degrees.zero();

    // private SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         Volts.of(0.25d).per(Second), 
    //         Volts.of(0.25d),
    //         null
    //     ), 
    //     new SysIdRoutine.Mechanism(
    //         volts -> setVoltage(volts), 
    //         log -> {
    //             log.motor("turret")
    //             .voltage(Robot.isSimulation() ? sim.getAppliedVoltage() : Volts.of(motor.getAppliedVoltage()))
    //             .angularPosition(Robot.isSimulation() ? sim.getMechanismAngle() : Rotations.of(motor.getRotations()))
    //             .angularVelocity(Robot.isSimulation() ? sim.getMechanismVelocity() : RotationsPerSecond.of(motor.getVelocityRPS()));
    //         },
    //         this
    //     )
    // );

    public TurretSubsystem(boolean enabled) {
        super(enabled);
        if (!enabled) return;
        // leftMotor = new MotorIOTalonFX(kLeftPortConfigs);
        rightMotor = new MotorIOTalonFX(kRightPortConfigs);
        
        // leftMotor.withPIDGains(kPositionGains);
        rightMotor.withPIDGains(kPositionGains);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);

        rightMotor.setSoftLimits(-2, 2);
        rightMotor.setDynamicMotionMagicSpeeds(20, 20);
        rightMotor.withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(20)
        );

        rightMotor.resetEncoderPosition(0);
        // rightMotor.withFeedbackConfigs(new FeedbackConfigs().withSensorToMechanismRatio(1d / kGearRatio));

        // leftMotor.setSoftLimits(kConverter.toMotor(kMinAngle).in(Rotations), kConverter.toMotor(kMaxAngle).in(Rotations));
        // rightMotor.setSoftLimits(kConverter.toMotor(kMinAngle).in(Rotations), kConverter.toMotor(kMaxAngle).in(Rotations));
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

    // @SuppressWarnings("unused")
    // private void setVoltage(Voltage volts) {
    //     leftMotor.setVoltage(volts.in(Volts));
    // }

    public Pose3d getLeftTurretPose(Pose3d swervePose) {
        return swervePose.transformBy(kLeftTurretToRobot);
    }

    public Pose3d getRightTurretPose(Pose3d swervePose) {
        return swervePose.transformBy(kRightTurretToRobot);
    }

    public Transform3d getTransformToTarget(Pose3d swervePose, Pose3d targetPose) {
        return getLeftTurretPose(swervePose).minus(targetPose);
    }

    @Override
    public void periodicTelemetry() {
        // logger.log("Left/Motor Rotations", leftMotor.getRotations());
        // logger.log("Left/Turret Degrees", getLeftTurretAngle().in(Degrees));
        // logger.log("Left/Target Degrees", getLeftTargetAngle().in(Degrees));
        // logger.log("Left/At Target", atLeftTargetAngle());

        logger.log("Right/Motor Rotations", rightMotor.getRotations());
        SmartDashboard.putNumber("Right Rotations", rightMotor.getRotations());
        // logger.log("Right/Turret Degrees", getRightTurretAngle().in(Degrees));
        // logger.log("Right/Target Degrees", getRightTargetAngle().in(Degrees));
        // logger.log("Right/At Target", atRightTargetAngle());
    }

    public Command setLeftTurretCommand(double targetDegrees) {
        return run(() -> {
            leftMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        });
    }

    public Command setRightTurretCommand(double targetDegrees) {
        return run(() -> {
            rightMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        });
    }

    public Command setRightRotations(double targetRotations) {
        return run(() -> {
            rightMotor.setPosition(targetRotations);
        });
    }

    public Command setRotations(double rots) {
        return run(() -> {
            rightMotor.setPosition(rots);
        });
    }

    public Command stopCommand() {
        return run(() -> {
            rightMotor.setPercentOutput(0);
        });
    }

    public Command driveManualCommand(double left, double right) {
        return run(() -> {
            // leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }

    public Command aimCommand(Supplier<Pose2d> swervePose, Supplier<Pose2d> targetPose) {
        return this.run(() -> {
            Pose3d leftPose = new Pose3d(swervePose.get()).transformBy(kLeftTurretToRobot);
            Pose3d rightPose = new Pose3d(swervePose.get()).transformBy(kRightTurretToRobot);

            Transform3d leftTransformToTarget = new Pose3d(targetPose.get()).minus(leftPose);
            Transform3d rightTransformToTarget = new Pose3d(targetPose.get()).minus(rightPose);

            double leftTurretAngle = Math.toDegrees(Math.atan(leftTransformToTarget.getX() / leftTransformToTarget.getY())) - swervePose.get().getRotation().getDegrees();
            double rightTurretAngle = Math.toDegrees(Math.atan(rightTransformToTarget.getX() / rightTransformToTarget.getY())) - swervePose.get().getRotation().getDegrees();

            leftTurretAngle = Utils.clamp(leftTurretAngle, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));
            rightTurretAngle = Utils.clamp(rightTurretAngle, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));

            leftMotor.setPosition(kConverter.toMotor(Degrees.of(leftTurretAngle)).in(Rotations));
            rightMotor.setPosition(kConverter.toMotor(Degrees.of(rightTurretAngle)).in(Rotations));
        });
    }
}