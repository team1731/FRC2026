package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.frc1731.hardware.camera.*;
import frc.lib.frc1731.math.Vector2d;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.drive.generated.*;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.drive.SwerveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

public class SwerveSubsystem extends BaseSubsystem {
    private CommandSwerveDrivetrain drivetrain = null;
    private Limelight limelight = null;
    private QuestNav quest = null;

    private SwerveDrivePoseEstimator estimator;

    private double fieldCentricHeading = 0;
    private double robotCentricHeading = 0;

    private Timer poseResetTimer = new Timer();

    private final ProfiledPIDController headingPID = kHeadingGains.toProfiledPIDController(kMaxAngularRate, kMaxAngularAcceleration);
    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public SwerveSubsystem(boolean enabled) {
        super(enabled);
        if(!enabled) return;

        this.drivetrain = TunerConstants.createDrivetrain();
        driveAtTargetControl.HeadingController.setPID(10,0,0);
	    driveAtTargetControl.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);

        this.estimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            Rotation2d.fromDegrees(getYaw()),
            drivetrain.getState().ModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5))
        );

        if (kUseLimelight) {
            this.limelight = new Limelight(kLimelightName);
            this.limelight.setLimelightPosition(kRobotToLimelight);
        }

        if (kUseVSLAM) {
            quest = new QuestNav();
        }

        if (kTelemetrize) {
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        configureAutoBuilder();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public void updateOdometry() {
        if (kUseLimelight) {
            limelight.updateOrientation(drivetrain.getPigeon2());
            LimelightHelpers.PoseEstimate estimate = limelight.getPoseEstimate();
            
            // if our angular velocity is greater than 720 degrees per second or we do not see any tags, ignore vision updates
            if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720 || estimate.tagCount == 0) return;
            Matrix<N3, N1> stdev = kSingleTagStdDevs;
            if (estimate.tagCount > 1) stdev = kMultiTagStdDevs;
            estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdev);

            if (Robot.IS_DISABLED.getAsBoolean()) { // Only reset the robot pose in disabled mode
                resetPose(estimate.pose);
            }
        }

        if (kUseVSLAM) {
            PoseFrame[] frames = quest.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : frames) {
                // Make sure the Quest was tracking the pose for this frame
                if (questFrame.isTracking()) {
                    // Get the pose of the Quest
                    Pose3d questPose = questFrame.questPose3d();
                    // Get timestamp for when the data was sent
                    double timestamp = questFrame.dataTimestamp();

                    // Transform by the mount pose to get your robot pose
                    Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());

                    // Add the measurement to our estimator if 20 ms has passed since last update
                    if (poseResetTimer.hasElapsed(0.2)) {
                        addVisionMeasurement(robotPose.toPose2d(), timestamp, kVSLAMStdDevs);
                        poseResetTimer.reset();
                    }
                }
            }
        }
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        estimator.resetPose(pose);
    }

    public double getYaw() {
        return drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }

    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public ChassisSpeeds getWheelSpeeds() {
        return getState().Speeds;
    }

    /*
     * This method will get called in two instances:
     * 1. After the VSLAM connects successfully
     * 2. When the alliance changes
     */
    public void configureInitialPosition() {
        System.out.println("CommandSwerveDrivetrain: configuring a new position");

	    Pose2d startingConfiguration = Robot.isRedAlliance()? 
            new Pose2d(10.38, 3.01, new Rotation2d(0)) : 
            new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
        resetPose(startingConfiguration);

        Rotation2d operatorPerspective = Robot.isRedAlliance()? 
                new Rotation2d(Math.toRadians(180)) : 
                new Rotation2d(Math.toRadians(0));
        drivetrain.setOperatorPerspectiveForward(operatorPerspective);
    }

    public void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getCurrentPose,
                this::resetAutoPose,
                this::getWheelSpeeds,
                // Consumer of ChassisSpeeds to drive the robot
                (speeds, feedsforwards)->drivetrain.setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                    kPPConstants,
                    kPPConstants
                ), RobotConfig.fromGUISettings(),
                () -> Robot.isRedAlliance(),
                this);
        } catch(Exception e) {
            System.out.println("CommandSwerveDrivetrain error - failed to configure auto bindings");
        }
    }

    /**
     * Only reset the pose of the robot if we are in simulation or VSLAM not connected, 
     * otherwise the Oculus will read current position
     */
    private void resetAutoPose(Pose2d pose) {
        if (Robot.isSimulation() || !kUseVSLAM) {
            this.resetPose(pose);
        }
    }

    @Override
    public void periodicTelemetry() {
        drivetrain.periodic();
        updateOdometry();

        logger.log("Current Pose", getCurrentPose());
        logger.log("Current Speeds", getWheelSpeeds());

        if (kUseVSLAM) {
            logger.log("VSLAM/Is Connected", quest.isConnected());
            logger.log("VSLAM/Is Tracking", quest.isTracking());
        }

        if (kUseLimelight) {
            logger.log("Limelight/Estimated Pose", estimator.getEstimatedPosition());
            logger.log("Limelight/Limelight Pose", limelight.getPoseEstimate().pose);
            logger.log("Limelight/Bot Pose", limelight.getBotPose());
        }

        Robot.kFieldLayout.setSimulatedRobotPose(getCurrentPose());
    }

    public Command driveCommand(CommandXboxController m_xboxController, BooleanSupplier isFieldCentric) {
        return run(() -> {
            Translation2d RotationCenter =  new Translation2d();

            if(m_xboxController.getHID().getLeftStickButton()){ // might need to flip X and Y due to field begin Y,X
                fieldCentricHeading = Math.toDegrees(Math.atan2(m_xboxController.getLeftX(),  m_xboxController.getLeftY())); // desired heading in field centric
                robotCentricHeading = drivetrain.getState().Pose.getRotation().getDegrees() - fieldCentricHeading; // current robot rotation in degrees
                if(robotCentricHeading >= 0 && robotCentricHeading < 90){ // between 0 and 90
                    RotationCenter = new Translation2d( 0.3, 0.3);
                } else if(robotCentricHeading >= 90 && robotCentricHeading < 180){ // between 90 and 180
                    RotationCenter = new Translation2d( 0.3, -0.3);
                } else if(robotCentricHeading >= 180 && robotCentricHeading < 270){ // between 180 and 270
                    RotationCenter = new Translation2d( -0.3, -0.3);
                } else if(robotCentricHeading >= 270 && robotCentricHeading < 0){ // between 270 and 360
                    RotationCenter = new Translation2d( -0.3, 0.3);
                } else {
                    RotationCenter = new Translation2d(0, 0);
                }
            } else {
                RotationCenter = new Translation2d(0, 0);
            }

            if ((Math.abs(m_xboxController.getLeftY()) < kDeadband) && 
                (Math.abs(m_xboxController.getLeftX()) < kDeadband) &&
                (Math.abs(m_xboxController.getRightX()) < kDeadband)){
                    drivetrain.setControl(brake);
            } else {
                if (isFieldCentric.getAsBoolean()) {
                    drivetrain.setControl(
                        kFieldCentricControl
                            .withVelocityX((Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                            .withVelocityY((Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                            .withCenterOfRotation(RotationCenter)
                    );
                }
                else {
                    drivetrain.setControl(
                        kRobotCentricControl
                            .withVelocityX((Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                            .withVelocityY((Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                            .withCenterOfRotation(RotationCenter)
                    );
                }
            }
        }).withName("Drive" + (isFieldCentric.getAsBoolean() ? "FieldCentric" : "RobotCentric"));
    }

    public Command pathfindToPoseCommand(Pose2d targetPose, double endVelocity) {
        return AutoBuilder.pathfindToPose(
            targetPose, 
            kPathfinderConstraints, 
            endVelocity
        ).withName("PathFindToPose");
    }

    public Command setHeadingTargetCommand(CommandXboxController m_xboxController, Rotation2d targetHeading) {
        return this.run(() -> {
            drivetrain.setControl(
                kFieldCentricControl
                    .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                    .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                    .withRotationalRate(headingPID.calculate(drivetrain.getRotation3d().getZ() % 360d, targetHeading.getDegrees()))
                    .withCenterOfRotation(new Translation2d(0, 0))
            );
        }).withName("SetHeadingTarget");
    }

    public Command facePoseCommand(CommandXboxController m_xboxController, Supplier<Pose2d> targetPose) {
        return this.run(() -> {
            Pose2d curPose = getCurrentPose();

            Vector2d robotToTarget = new Vector2d(curPose.getX() - targetPose.get().getX(), curPose.getY() - targetPose.get().getY());

            drivetrain.setControl(
                kFieldCentricControl
                    .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                    .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                    .withRotationalRate(headingPID.calculate(drivetrain.getRotation3d().getZ() % 360d, robotToTarget.getDirection().getDegrees()))
                    .withCenterOfRotation(new Translation2d(0, 0))
            );
        }).withName("FacePose");
    }
}