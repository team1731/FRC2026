package frc.robot.subsystems.swerve;
 
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveRequests.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotState;
import frc.lib.frc1678.util.Util;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.generated.*;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionHandler;

/**
 * Command-based wrapper around the CTRE generated swerve drivetrain.
 *
 * <p>This subsystem owns drivetrain hardware, applies joystick/auto requests, configures
 * PathPlanner, and forwards vision measurements into the CTRE pose estimator.
 */
public class SwerveSubsystem extends BaseSubsystem {
    // Hardware
    private SwerveDriveState state = new SwerveDriveState();
    private CommandSwerveDrivetrain drivetrain;

    /** Vision subsystem helper used for AprilTag/VSLAM pose fusion and simulation resets. */
    public VisionHandler handler; // public so robot.java can reset VSLAM pose

    private double snailModeScalar = 1.0;

    private double targetError = 0;

    /** Combined simulated battery current for all drive and steer controllers. */
    public double getSimSupplyCurrent() {
        double total = 0.0;
        for (var module : drivetrain.getModules()) {
            total += module.getDriveMotor().getSimState().getSupplyCurrent();
            total += module.getSteerMotor().getSimState().getSupplyCurrent();
        }
        return total;
    }

    public SwerveSubsystem() {
        if (!isActiveSubsystem()) return; // Build nothing if the subsystem is turned off
        this.drivetrain = TunerConstants.createDrivetrain();
        this.handler = new VisionHandler(
            (pose, timestamp, estimationStdDevs) -> 
                this.drivetrain.addVisionMeasurement(pose, timestamp, estimationStdDevs), 
                VisionConstants.getAprilTagIOs()
        );

        this.setDefaultCommand(this.joystickDrive());

        try {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                // Consumer of ChassisSpeeds to drive the robot
                (speeds, feedforwards)-> this.drivetrain.setControl(
                    kAutoRequest.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    kPPConstants,
                    kPPConstants
                ), RobotConfig.fromGUISettings(),
                () -> Robot.isRedAlliance(),
                this);
        } catch(Exception e) {
            System.out.println("SwerveSubsystem error - failed to configure auto bindings");
        }
    }

    /**
     * Returns the current field-relative drivetrain pose.
     *
     * @return estimated robot pose on the field
     */
    public Pose2d getPose() {
        return this.state.Pose;
    }

    /**
     * Returns current robot-relative chassis speeds from the drivetrain state.
     *
     * @return current chassis speeds
     */
    public ChassisSpeeds getSpeeds() {
        return this.state.Speeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() { // used for shoot on the fly
        return new ChassisSpeeds(
            getSpeeds().vxMetersPerSecond * getPose().getRotation().getCos()
                    - getSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
            getSpeeds().vyMetersPerSecond * getPose().getRotation().getCos()
                    + getSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
            getSpeeds().omegaRadiansPerSecond);
    }

    /**
     * Returns current Pigeon yaw in degrees.
     *
     * @return drivetrain yaw in degrees
     */
    public double getYaw() {
        return this.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }

    /**
     * Applies a stator current limit to every swerve drive motor.
     *
     * @param limit stator current limit in amps
     */
    public void setStatorCurrentLimit(double limit) {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : drivetrain.getModules()) {
            // Create config object
            TalonFXConfigurator configurator = module.getDriveMotor().getConfigurator();
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    
            // Dynamically change limit based on logic (e.g., set to 40A)
            currentLimits.StatorCurrentLimit = limit;
            currentLimits.StatorCurrentLimitEnable = true;
    
            // Apply configuration
            configurator.apply(currentLimits);
        }
    }

    /**
     * Re-seeds CTRE field-centric driving from the current robot heading.
     */
    public void seedFieldCentric() {
        this.drivetrain.seedFieldCentric();
    }

    /**
     * Resets drivetrain pose estimation to a known field pose.
     *
     * @param pose new robot field pose
     */
    public void resetPose(Pose2d pose) {
        this.drivetrain.resetPose(pose);
    }

    public void setSnailMode(boolean snail) {
        this.snailModeScalar = snail ? 0.5 : 1.0;
    }

    /**
     * Updates drivetrain state, CTRE periodic handling, vision fusion, and swerve logs.
     */
    @Override
    public void periodicTelemetry() {
        this.state = this.drivetrain.getState();
        RobotState.updateDrive(state.Pose);
        this.drivetrain.periodic();
        this.handler.periodic(
            getPose(),
            getPose().getRotation().getDegrees(), 
            drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()
        );

        super.logger.log("Current Pose", getPose());
        super.logger.log("Current Speeds", getSpeeds());

        Robot.field.setRobotPose(getPose());
    }

    /**
     * Sends the desired command to the swerve
     * @param request a supplier for a swerve command for either movement or braking
     */
    private Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.drivetrain.setControl(request.get()));
    }

    /**
     * Drives the swerve base using controller joystick input
     */
    public Command joystickDrive() {
        return run(() -> {
            double velX = kXScalar.scale(-Controls.getDriver().getLeftY() * SwerveConstants.kMaxSpeed * SwerveConstants.kTranslationScalar);
            double velY = kYScalar.scale(-Controls.getDriver().getLeftX() * SwerveConstants.kMaxSpeed * SwerveConstants.kTranslationScalar);
            double omega = kOmegaScalar.scale(-Controls.getDriver().getRightX() * SwerveConstants.kMaxAngularRate * SwerveConstants.kRotationScalar);

            velX *= snailModeScalar;
            velY *= snailModeScalar;
            omega *= snailModeScalar;

            if (velX == 0.0 && velY == 0.0 && omega == 0.0) {
                this.drivetrain.setControl(kBrakeRequest);
            } else {
                this.drivetrain.setControl(
                    kJoystickFieldCentricRequest
                        .withVelocityX(velX)
                        .withVelocityY(velY)
                        .withRotationalRate(omega)
                );
            }
        });
    }

    public Command joystickTargetLock(Supplier<Translation2d> target) {
        return run(() -> {
            Pose2d curPose = getPose();
            Translation2d robotTranslation = curPose.getTranslation();
            Translation2d targetTranslation = target.get();
            // Angle from robot to target
            Rotation2d targetAngle = targetTranslation.minus(robotTranslation).getAngle();
            // Flip by 180 degrees so the BACK of the robot points at the target
            Rotation2d desiredAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
            double rotRate = kHeadingCtrl.calculate(curPose.getRotation().getRadians() % (2 * Math.PI), desiredAngle.getRadians());

            double velX = kXScalar.scale(-Controls.getDriver().getLeftY() * SwerveConstants.kMaxSpeed * SwerveConstants.kTranslationScalar * 0.5);
            double velY = kYScalar.scale(-Controls.getDriver().getLeftX() * SwerveConstants.kMaxSpeed * SwerveConstants.kTranslationScalar * 0.5);
            
            drivetrain.setControl(
                kJoystickFieldCentricRequest
                    .withVelocityX(velX)
                    .withVelocityY(velY)
                    .withRotationalRate(rotRate)
            );
        }).withName("JoystickTargetLock");
    }

    /**
     * Calculates and runs a trajectory to hit desired end point while avoiding known field obstacles
     * @param setpoint target position to drive the swerve towards. Flips pose for red alliance.
     * 
     * @implNote Trying out {@code LocalADStarAK} since it is highly recommended. Cannot guarentee results but I bet it'll be fine :)
     */
    public Command pathfindToAlliancePose(Pose2d setpoint) {
        return AutoBuilder.pathfindToPoseFlipped(setpoint, kPathfinderConstraints);
    }

    /**
     * Calculates and runs a trajectory to hit desired end point while avoiding known field obstacles
     * @param setpoint target position to drive the swerve towards. DOES NOT flip pose for red alliance.
     * 
     * @implNote Trying out {@code LocalADStarAK} since it is highly recommended. Cannot guarentee results but I bet it'll be fine :)
     */
    public Command pathfindToPose(Pose2d setpoint) {
        return AutoBuilder.pathfindToPose(setpoint, kPathfinderConstraints);
    }

    /**
     * Auto-aligns in X and Y simultaneously while facing the setpoint rotation.
     *
     * @param setpoint target field pose to drive toward
     * @return command that continuously drives toward the requested pose
     */
    public Command autoAlignXY(Pose2d setpoint) {
        return this.applyRequest(() -> 
            kAutoAlignRequest
                .withVelocityX(-kAutoAlignDriveCtrl.calculate(state.Pose.getX(), setpoint.getX()))
                .withVelocityY(-kAutoAlignStrafeCtrl.calculate(state.Pose.getY(), setpoint.getY()))
                .withTargetDirection(setpoint.getRotation())
        );
    }

    /**
     * Auto-aligns by correcting Y first, then correcting X while holding target heading.
     *
     * @param setpoint supplier for the target pose, allowing alliance-aware targets
     * @return command that drives toward the supplied target pose
     */
    public Command autoAlignYThenX(Supplier<Pose2d> setpoint) {
        return this.applyRequest(() -> 
            kAutoAlignRequest
                .withVelocityY(-kAutoAlignStrafeCtrl.calculate(getPose().getY(), setpoint.get().getY()))
                .withTargetDirection(setpoint.get().getRotation())
        ).until(() -> Util.isWithin(setpoint.get().getY(), getPose().getY(), 0.05))
        .andThen(this.applyRequest(() -> 
            kAutoAlignRequest
                .withVelocityX(-kAutoAlignDriveCtrl.calculate(getPose().getX(), setpoint.get().getX()))
                .withVelocityY(-kAutoAlignStrafeCtrl.calculate(getPose().getY(), setpoint.get().getY()))
                .withTargetDirection(setpoint.get().getRotation())
        ))
        .withName("AutoAlignYThenX");
    }

    public Command lockHeadingTarget(Supplier<Translation2d> target) {
        return run(() -> {
            Pose2d curPose = getPose();
            Translation2d robotTranslation = curPose.getTranslation();
            Translation2d targetTranslation = target.get();
            // Angle from robot to target
            Rotation2d targetAngle = targetTranslation.minus(robotTranslation).getAngle();
            // Flip by 180 degrees so the BACK of the robot points at the target
            Rotation2d desiredAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
            double rotRate = kHeadingCtrl.calculate(curPose.getRotation().getRadians() % (2 * Math.PI), desiredAngle.getRadians());
            
            targetError = targetAngle.minus(desiredAngle).getDegrees();
            drivetrain.setControl(
                kJoystickFieldCentricRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotRate)
            );
        }).withName("LockHeading")
        .until(() -> Math.abs(targetError) < 1.0)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}
