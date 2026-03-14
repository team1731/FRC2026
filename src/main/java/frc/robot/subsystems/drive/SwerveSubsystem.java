package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.frc1731.math.Vector2d;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.drive.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.generated.TunerConstants;

import static frc.robot.subsystems.drive.SwerveConstants.*;

public class SwerveSubsystem extends BaseSubsystem {
    private CommandSwerveDrivetrain drivetrain;
    private Telemetry telemetry;
  
    private PIDController headingPID = kHeadingGains.toPIDController();

    public SwerveSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    public void initializeHardware() {
        this.drivetrain = TunerConstants.createDrivetrain();
        if (kShouldTelemetrize) {
            telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        configureAutoBuilder();
        configureInitialPosition();
    }

    private void configureAutoBuilder() {
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

    /*
     * This method will get called in two instances:
     * 1. After the VSLAM connects successfully
     * 2. When the alliance changes
     */
    private void configureInitialPosition() {
        System.out.println("CommandSwerveDrivetrain: configuring a new position");

        // Rotation2d operatorPerspective = Robot.isRedAlliance() ? 
        //         new Rotation2d(Math.toRadians(180)) : 
        //         new Rotation2d(Math.toRadians(0));
        // drivetrain.setOperatorPerspectiveForward(operatorPerspective);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /**
     * Only reset the pose of the robot is we are in simulation or VSLAM not connected, 
     * otherwise the Oculus will read current position
     */
    private void resetAutoPose(Pose2d pose) {
        this.resetPose(pose);
    }

    public void resetHeadingButtonPressed() {
        drivetrain.seedFieldCentric();
        drivetrain.getPigeon2().reset();
    }

    public void resetJustHeading(Pose2d autoStartPose) {  // this is called from auto preloads
        resetPose(new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), autoStartPose.getRotation()));
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

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {    // used for shoot on the fly
        return new ChassisSpeeds(
            getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getCos()
                    - getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getSin(),
            getWheelSpeeds().vyMetersPerSecond * getCurrentPose().getRotation().getCos()
                    + getWheelSpeeds().vxMetersPerSecond * getCurrentPose().getRotation().getSin(),
            getWheelSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void periodicTelemetry() {
        drivetrain.periodic();
    }

    public Command driveCommand(CommandXboxController m_xboxController, BooleanSupplier isFieldCentric) {
        return run(() -> {
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
                    );
                } 
                 else {
                     drivetrain.setControl(
                         kRobotCentricControl
                             .withVelocityX(-(Math.abs(m_xboxController.getLeftY()) * m_xboxController.getLeftY()) * kMaxSpeed)
                             .withVelocityY(-(Math.abs(m_xboxController.getLeftX()) * m_xboxController.getLeftX()) * kMaxSpeed)
                             .withRotationalRate(-m_xboxController.getRightX() * kMaxAngularRate)
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