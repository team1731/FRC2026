package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import static frc.robot.subsystems.vision.limelight.AprilTagConstants.*;
import static frc.robot.subsystems.vision.questnav.VSLAMConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.frc1731.hardware.camera.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.drive.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.vision.limelight.AprilTagConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.drive.SwerveConstants.*;

public class SwerveSubsystem extends BaseSubsystem {
    private CommandSwerveDrivetrain drivetrain;

    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    private double fieldCentricHeading = 0.0;
    private double robotCentricHeading = 0.0;

    private Timer GoodLimelightTimer = new Timer();
  //  private boolean hadgoodLimelight = false;
    private boolean visionCheckingHasStarted = false;
    private Pose2d baselinePose;
    private boolean hasGoodOdometry = false;
    private double distanceBetweenPoses;
    private boolean headingIsInitialized = false;


    private QuestNav questNav;

    private static boolean isQuestSeeded = false;

        private static Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    private Timer questPoseResetTimer = new Timer();  // ussed primarily in dissabled to reset the pose every few seconds in case the robot is moved b4 auto

    private final ProfiledPIDController headingPID = kHeadingGains.toProfiledPIDController(kMaxAngularRate, kMaxAngularAcceleration);
 


    public SwerveSubsystem(boolean enabled) {
        super(enabled);
        if(!enabled) return;
        questNav = new QuestNav();

        this.drivetrain = TunerConstants.createDrivetrain();
        driveAtTargetControl.HeadingController.setPID(10,0,0);
	    driveAtTargetControl.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
        // this.drivetrain.resetRotation(new Rotation2d());

        Transform3d tf = kCameraToRobot;
        LimelightHelpers.setCameraPose_RobotSpace(kLimelightName, tf.getX(), tf.getY(), tf.getZ(), tf.getRotation().getX(), tf.getRotation().getY(), tf.getRotation().getZ());

        configureAutoBuilder();
        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public void updateVisionOdometry() {
        // double orientation = Robot.isRedAlliance() ? 180 + drivetrain.getPigeon2().getYaw().getValueAsDouble() : drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double orientation = drivetrain.getPigeon2().getYaw().getValueAsDouble();// + drivetrain.getOperatorForwardDirection().getDegrees();
        // double orientation = 180 - getCurrentPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(kLimelightName, orientation, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(AprilTagConstants.kLimelightName);
        
        // Do not use estimate if we are rotating too fast or if we see less than 2 tags
        if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720 || mt2.tagCount <= 1) return;

        this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.5,.5,999999));
        
        if (!visionCheckingHasStarted) {
            GoodLimelightTimer.restart();
            baselinePose = mt2.pose;
            visionCheckingHasStarted = true;
        } else if (GoodLimelightTimer.hasElapsed(2)) {
            distanceBetweenPoses = distanceBetween(mt2.pose, baselinePose);
            // SmartDashboard.putNumber("distanceBetweenPoses", distanceBetweenPoses);
            // SmartDashboard.putBoolean("hasGoodOdometry", hasGoodOdometry);
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
        resetQuestPose(new Pose3d(pose));  
        isQuestSeeded = true; 
        drivetrain.getPigeon2().setYaw(drivetrain.getOperatorForwardDirection().getDegrees());
    }

    public void resetHeadingButtonPressed() {
        Pose2d resetPosition = Robot.isRedAlliance()
                ? new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(180)))
                : new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(0)));
        resetPose(resetPosition);
        drivetrain.getPigeon2().setYaw(drivetrain.getOperatorForwardDirection().getDegrees());
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
        this.resetPose(pose);
    }

    @Override
    public void periodic () {
        super.periodic();
        drivetrain.periodic();
        // if(!headingIsInitialized) {
        //      Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(0)))
        //         : new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), new Rotation2d(Math.toRadians(180)));
        //         resetPose(resetPosition);
        // }

        updateVisionOdometry();
        questNav.commandPeriodic();
        SmartDashboard.putBoolean("isSeeded", isQuestSeeded);
        SmartDashboard.putBoolean("isTracking", questNav.isTracking());
        SmartDashboard.putBoolean("isConnected", questNav.isConnected());
        SmartDashboard.putBoolean("hasgoodtracking",hasGoodOdometry());
        SmartDashboard.putNumber("Gyro Yaw",drivetrain.getPigeon2().getYaw().getValueAsDouble());
      //  if (!isQuestSeeded || (DriverStation.isDisabled() && questPoseResetTimer.hasElapsed(5))) {
      //      if (hasGoodOdometry() && questNav.isTracking() && questNav.isConnected()) {
       //         this.resetQuestPose(new Pose3d(getCurrentPose()));
      //          questPoseResetTimer.restart();
                isQuestSeeded = true;
       //     }
       // }
        if ( !questNav.isTracking() ) {
            isQuestSeeded = false;
        }
        if (questNav.isTracking() && isQuestSeeded && questNav.isConnected()) {
            addQuestVisionMeasurement();
        }


    }

    @Override
    public void periodicTelemetry() {
       // drivetrain.periodic();
       // updateOdometry();

        // logger.log("Current Pose", getCurrentPose());
        // logger.log("Current Speeds", getWheelSpeeds());
        // logger.log("Target Speeds", targetSpeeds);
        // logger.log("Estimated Pose", getCurrentPose());

        // if(getQuestPose() != null) {
        //     logger.log("Questnav Estimated Pose", getQuestPose());
        // }

        // Robot.kFieldLayout.setSimulatedRobotPose(getCurrentPose());
    }

    public Command driveCommand(CommandXboxController m_xboxController, BooleanSupplier isFieldCentric) {
        return run(() -> {
            Translation2d RotationCenter =  new Translation2d();
            double speedMultiplier = m_xboxController.rightBumper().getAsBoolean() ? 0.5 : 1.0;

            // if(m_xboxController.getHID().getLeftStickButton()){ // might need to flip X and Y due to field begin Y,X
            //     fieldCentricHeading = Math.toDegrees(Math.atan2(m_xboxController.getLeftX(),  m_xboxController.getLeftY())); // desired heading in field centric
            //     robotCentricHeading = drivetrain.getState().Pose.getRotation().getDegrees() - fieldCentricHeading; // current robot rotation in degrees
            //     if(robotCentricHeading >= 0 && robotCentricHeading < 90){ // between 0 and 90
            //         RotationCenter = new Translation2d( 0.3, 0.3);
            //     } else if(robotCentricHeading >= 90 && robotCentricHeading < 180){ // between 90 and 180
            //         RotationCenter = new Translation2d( 0.3, -0.3);
            //     } else if(robotCentricHeading >= 180 && robotCentricHeading < 270){ // between 180 and 270
            //         RotationCenter = new Translation2d( -0.3, -0.3);
            //     } else if(robotCentricHeading >= 270 && robotCentricHeading < 0){ // between 270 and 360
            //         RotationCenter = new Translation2d( -0.3, 0.3);
            //     } else {
            //         RotationCenter = new Translation2d(0, 0);
            //     }
            // } else {
            //     RotationCenter = new Translation2d(0, 0);
            // }

            if ((Math.abs(m_xboxController.getLeftY()) < 0.05) && 
                (Math.abs(m_xboxController.getLeftX()) < 0.05) &&
                (Math.abs(m_xboxController.getRightX()) < 0.05)){
                    drivetrain.setControl(brake);
            } else {
                if (isFieldCentric.getAsBoolean()) {
                    drivetrain.setControl(
                        kFieldCentricControl
                            .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed * speedMultiplier)
                            .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed * speedMultiplier)
                            .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate * speedMultiplier)
                            .withCenterOfRotation(RotationCenter)
                    );
                } 
                 else {
                     drivetrain.setControl(
                         kRobotCentricControl
                             .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed * speedMultiplier)
                             .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed * speedMultiplier)
                             .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate * speedMultiplier)
                             .withCenterOfRotation(RotationCenter)
                     );
                 }
            }
        }).withName("Drive" + (isFieldCentric.getAsBoolean() ? "FieldCentric" : "RobotCentric"));
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {    // used for shoot on the fly
        return new ChassisSpeeds(
                getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getCos()
                        - getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getSin(),
                getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getCos()
                        + getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getSin(),
                getWheelSpeeds().omegaRadiansPerSecond);
    }




    // public Command driveToTargetCommand(CommandXboxController m_xboxController) {
    //     return run(() -> {
    //         double fieldCentricX = (Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY());
    //         double fieldCentricY = (Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX());

    //         if (Robot.isRedAlliance()) {
    //             fieldCentricX = fieldCentricX * -1;
    //             fieldCentricY = fieldCentricY * -1;
    //         }

    //         aprilTagTargetTracker.recalculateDriveFeedback(getCurrentPose(), fieldCentricX, fieldCentricY);

    //         if (aprilTagTargetTracker.hasVisibleTarget()) {
    //             lostTargetCount = 0;
    //             lockedOnce = true;
    //         } else {
    //             lostTargetCount++;
    //         }

    //         if (aprilTagTargetTracker.hasVisibleTarget() || ((lostTargetCount < 5) && lockedOnce == true)) {
    //             // note to self since this is confusing... Y becomes X and X becomes Y and that change happens earlier for driving at target
    //             drivetrain.setControl(
    //                     driveAtTargetControl.withVelocityX(aprilTagTargetTracker.getCalculatedX() * kMaxSpeed)
    //                             .withVelocityY(  aprilTagTargetTracker.getCalculatedY() *kMaxSpeed)
    //                             .withTargetDirection(aprilTagTargetTracker.getRotationTarget()));
    //         } else {
    //             drivetrain.setControl(
    //                     kFieldCentricControl
    //                             .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY())
    //                                     * kMaxSpeed)
    //                             .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX())
    //                                     * kMaxSpeed)
    //                             .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate));
    //         }
    //     }).withName("DriveToTarget");
    // }

  //  public Command pathfindToPoseCommand(Pose2d targetPose, double endVelocity) {
 //       return AutoBuilder.pathfindToPose(
 //           targetPose, 
 //           kPathfinderConstraints, 
 //           endVelocity
 //       ).withName("PathFindToPose");
//    }

 //   public Command setHeadingTargetCommand(CommandXboxController m_xboxController, Rotation2d targetHeading) {
  //      return this.run(() -> {
  //          drivetrain.setControl(
  //              kFieldCentricControl
  //                  .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
  //                  .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
  //                  .withRotationalRate(headingPID.calculate(drivetrain.getRotation3d().getZ() % 360d, targetHeading.getDegrees()))
  //                  .withCenterOfRotation(new Translation2d(0, 0))
  //          );
  //      }).withName("SetHeadingTarget");
  //  }

 //   public Command facePoseCommand(CommandXboxController m_xboxController, Supplier<Pose2d> targetPose) {
  //      return this.run(() -> {
  //          Pose2d curPose = getCurrentPose();
//
  //          Vector2d robotToTarget = new Vector2d(curPose.getX() - targetPose.get().getX(), curPose.getY() - targetPose.get().getY());
//
  //          drivetrain.setControl(
   //             kFieldCentricControl
  //                  .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
 //                   .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
 //                   .withRotationalRate(headingPID.calculate(drivetrain.getRotation3d().getZ() % 360d, robotToTarget.getDirection().getDegrees()))
 //                   .withCenterOfRotation(new Translation2d(0, 0))
 //           );
 //       }).withName("FacePose");
 //   }

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

                addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
                 
                }
                
            }
        }
    }

public boolean isVslamConnected() {
   return (questNav.isTracking()  && questNav.isConnected());
}
}