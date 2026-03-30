package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.frc1731.hardware.camera.limelight.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.drive.generated.*;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.drive.SwerveConstants.*;
import static frc.robot.subsystems.drive.VisionConstants.*;

public class SwerveSubsystem extends BaseSubsystem {
    private CommandSwerveDrivetrain drivetrain;

    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private Timer GoodLimelightTimer = new Timer();
    private boolean visionCheckingHasStarted = false;
    private Pose2d baselinePose;
    private boolean hasGoodOdometry = false;
    private double distanceBetweenPoses;
    private boolean headingIsInitialized = false;

    private QuestNav questNav;

    private static boolean isQuestSeeded = false;

    private Timer questPoseResetTimer = new Timer();  // ussed primarily in dissabled to reset the pose every few seconds in case the robot is moved b4 auto

    private final ProfiledPIDController headingPID = kHeadingGains.toProfiledPIDController(kMaxAngularRate, kMaxAngularAcceleration);

    public SwerveSubsystem(boolean enabled) {
        super(enabled);
        if(!enabled) return;
        questNav = new QuestNav();

        this.drivetrain = TunerConstants.createDrivetrain();
        driveAtTargetControl.HeadingController.setPID(10,0,0);
	    driveAtTargetControl.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);

        Transform3d tf = kLimelightToRobot;
        LimelightHelpers.setCameraPose_RobotSpace(kLimelightName, tf.getX(), tf.getY(), tf.getZ(), tf.getRotation().getX(), tf.getRotation().getY(), tf.getRotation().getZ());

        configureAutoBuilder();
        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public void updateVisionOdometry() {
        // double orientation = Robot.isRedAlliance() ? 180 - getCurrentPose().getRotation().getDegrees() : getCurrentPose().getRotation().getDegrees();
        double orientation = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation(kLimelightName, orientation, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
        
        // Do not use estimate if we are rotating too fast or if we see less than 2 tags
        if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720 || mt2.tagCount <= 1) return;

        // this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.5,.5,2));
        
        if (!visionCheckingHasStarted) {
            GoodLimelightTimer.restart();
            baselinePose = mt2.pose;
            visionCheckingHasStarted = true;
        } else if (GoodLimelightTimer.hasElapsed(2)) {
            distanceBetweenPoses = distanceBetween(mt2.pose, baselinePose);
            if (distanceBetweenPoses < .005) {
                hasGoodOdometry = true;                 
            } else {
                hasGoodOdometry = false;
            }
            visionCheckingHasStarted = false;
        }
    }

    public static double distanceBetween(Pose2d a, Pose2d b) { Translation2d ta = a.getTranslation(); Translation2d tb = b.getTranslation(); return ta.getDistance(tb); }
    
    public boolean hasGoodOdometry() {
        return hasGoodOdometry;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        // drivetrain.getPigeon2().setYaw(pose.getRotation().getDegrees());
        resetQuestPose(new Pose3d(pose));  
        isQuestSeeded = true; 
    }

    public void resetHeadingButtonPressed() {
        Pose2d resetPosition = Robot.isRedAlliance()
                ? new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(180)))
                : new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(0)));
        resetPose(resetPosition);

    }

    public void resetJustHeading(Pose2d autoStartPose) {  // this is called from auto preloads
        Pose2d resetPosition =  new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), autoStartPose.getRotation());
                resetPose(resetPosition);

        
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

	   // Pose2d startingConfiguration = Robot.isRedAlliance()? 
       //     new Pose2d(10.38, 3.01, new Rotation2d(0)) : 
       //     new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
       // resetPose(startingConfiguration);

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
     * Only reset the pose of the robot is we are in simulation or VSLAM not connected, 
     * otherwise the Oculus will read current position
     */
    private void resetAutoPose(Pose2d pose) {
        if (Robot.isSimulation() || !kUseVSLAM) {
          //  this.resetPose(pose);  we probably do not want to do this if VSLAM and limelight are working?
        }
        // this.resetPose(pose);
    }

    @Override
    public void periodic () {
        super.periodic();

        drivetrain.periodic();
        updateVisionOdometry();
        questNav.commandPeriodic();
        SmartDashboard.putBoolean("isSeeded", isQuestSeeded);
        SmartDashboard.putBoolean("isTracking", questNav.isTracking());
        SmartDashboard.putBoolean("isConnected", questNav.isConnected());
        SmartDashboard.putBoolean("hasgoodtracking",hasGoodOdometry());
    //    if (!isQuestSeeded || (DriverStation.isDisabled() && questPoseResetTimer.hasElapsed(5))) {
    //        if (hasGoodOdometry() && questNav.isTracking() && questNav.isConnected()) {
    //            this.resetQuestPose(new Pose3d(getCurrentPose()));
    //            questPoseResetTimer.restart();
        isQuestSeeded = true;
    //        }
    //    }

        if (!questNav.isTracking() ) {
            isQuestSeeded = false;
        }
        if (questNav.isTracking() && isQuestSeeded && questNav.isConnected()) {
            addQuestVisionMeasurement();
        }
    }

    @Override
    public void periodicTelemetry() {
        drivetrain.periodic();
        questNav.commandPeriodic();

        updateVisionOdometry();

        SmartDashboard.putBoolean("isSeeded", isQuestSeeded);
        SmartDashboard.putBoolean("isTracking", questNav.isTracking());
        SmartDashboard.putBoolean("isConnected", questNav.isConnected());
        SmartDashboard.putBoolean("hasgoodtracking", hasGoodOdometry());

        if (!questNav.isTracking()) {
            isQuestSeeded = false;
        } else {
            isQuestSeeded = true;
        }

        if (questNav.isTracking() && isQuestSeeded && questNav.isConnected()) {
            addQuestVisionMeasurement();
        }
    }

    public Command driveCommand(CommandXboxController m_xboxController, BooleanSupplier isFieldCentric) {
        return run(() -> {
            Translation2d RotationCenter =  new Translation2d();

            if ((Math.abs(m_xboxController.getLeftY()) < kDeadband) && 
                (Math.abs(m_xboxController.getLeftX()) < kDeadband) &&
                (Math.abs(m_xboxController.getRightX()) < kDeadband)){
                    drivetrain.setControl(brake);
            } else {
                if (isFieldCentric.getAsBoolean()) {
                    drivetrain.setControl(
                        kFieldCentricControl
                            .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                            .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                            .withCenterOfRotation(RotationCenter)
                    );
                } 
                 else {
                     drivetrain.setControl(
                         kRobotCentricControl
                             .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                             .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                             .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
                             .withCenterOfRotation(RotationCenter)
                     );
                 }
            }
        }).withName("Drive" + (isFieldCentric.getAsBoolean() ? "FieldCentric" : "RobotCentric"));
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() { // used for shoot on the fly
        return new ChassisSpeeds(
                getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getCos()
                        - getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getSin(),
                getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getCos()
                        + getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getSin(),
                getWheelSpeeds().omegaRadiansPerSecond);
    }

     public Pose3d getQuestPose() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());
            return robotPose;
        }
        return null;
    }

     public void resetQuestPose(Pose3d robotPose) {
        // Transform the robot pose by the mount pose to get the corresponding Quest pose
        Pose3d questResetPose = robotPose.transformBy(kRobotToOculus);

        // Set the QuestNav pose to the calculated Quest pose
        System.out.println("setting quest pose@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        questNav.setPose(questResetPose);
        questPoseResetTimer.reset();
    }

  public void addQuestVisionMeasurement() {
        questPoseResetTimer.start();

        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());

                if (questNav.isTracking()  && isQuestSeeded) {
                    addVisionMeasurement(robotPose.toPose2d(), timestamp, kQuestnavStdev);
                }
            }
        }
    }

public boolean isVslamConnected() {
   return (questNav.isTracking()  && questNav.isConnected());
}

@Override
protected void initializeHardware() {}
}