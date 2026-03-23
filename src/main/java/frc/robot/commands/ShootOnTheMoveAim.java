package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class ShootOnTheMoveAim extends Command {
  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem hopper;
  private Command armOscillateCommand;

  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;

  private int lockedTagId = -1;
  private Translation2d hubCenterField = null;
  private boolean currentlySeeingHubTag = false;

  public ShootOnTheMoveAim(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      HopperSubsytem hopper,
      DoubleSupplier xInput,
      DoubleSupplier yInput) {
    this.swerve = swerve;
    this.vision = swerve.getVision();
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.xInput = xInput;
    this.yInput = yInput;

    addRequirements(swerve, shooter, indexer, hopper);
  }

  @Override
  public void initialize() {
    lockedTagId = -1;
    currentlySeeingHubTag = false;
    hubCenterField = vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).orElse(null);
  }

  @Override
  public void execute() {
    Optional<Integer> visibleTagOpt = vision.getBestVisibleHubTag(Constants.HUB_TAG_IDS);
    currentlySeeingHubTag = visibleTagOpt.isPresent();

    if (visibleTagOpt.isPresent()) {
      lockedTagId = visibleTagOpt.get();
    }

    if (lockedTagId != -1) {
      vision.getHubCenterFieldPositionFromTag(lockedTagId).ifPresent(center -> hubCenterField = center);
    } else {
      vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).ifPresent(center -> hubCenterField = center);
    }

    Pose2d robotPose = swerve.getPose();

    if (hubCenterField == null) {
      ChassisSpeeds noTargetSpeeds =
          swerve.getTargetSpeeds(
              xInput.getAsDouble(),
              yInput.getAsDouble(),
              robotPose.getRotation());
      swerve.driveFieldOriented(noTargetSpeeds);
      shooter.set(0.0);
      indexer.setduty(0.0);
      hopper.setduty(0.0);
      return;
    }

    // Driver-requested translation, using current heading so we only estimate X/Y intent.
    ChassisSpeeds commandedFieldVelocity =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            robotPose.getRotation());

    ChassisSpeeds measuredFieldVelocity = swerve.getFieldVelocity();

    Translation2d commandedVel =
        new Translation2d(
            commandedFieldVelocity.vxMetersPerSecond,
            commandedFieldVelocity.vyMetersPerSecond);

    Translation2d measuredVel =
        new Translation2d(
            measuredFieldVelocity.vxMetersPerSecond,
            measuredFieldVelocity.vyMetersPerSecond);

    // Best estimate of release velocity.
    Translation2d estimatedVel =
        measuredVel.times(Constants.SHOT_MEASURED_VELOCITY_WEIGHT)
            .plus(commandedVel.times(1.0 - Constants.SHOT_MEASURED_VELOCITY_WEIGHT));

    // Predict where the robot will be when the note actually leaves.
    Translation2d predictedReleasePose =
        robotPose.getTranslation().plus(estimatedVel.times(Constants.SHOT_POSE_PREDICTION_SECS));

    Translation2d toHub = hubCenterField.minus(predictedReleasePose);
    double distanceMeters = toHub.getNorm();

    if (distanceMeters < 1e-6) {
      swerve.driveFieldOriented(new ChassisSpeeds());
      shooter.set(0.0);
      indexer.setduty(0.0);
      hopper.setduty(0.0);
      return;
    }

    double shooterRPM = shooter.rpmForDistanceMeters(distanceMeters);
    shooter.setMechanismVelocitySetpoint(RPM.of(shooterRPM));

    double ballSpeedMps = Constants.estimateBallSpeedMps(shooterRPM);

    Translation2d towardHubUnit =
        new Translation2d(toHub.getX() / distanceMeters, toHub.getY() / distanceMeters);

    // Required launch vector relative to robot:
    // launchVector + estimatedRobotVelocity = desiredBallVelocityTowardHub
    Translation2d launchVector =
        towardHubUnit.times(ballSpeedMps).minus(estimatedVel);

    if (launchVector.getNorm() < 1e-6) {
      launchVector = towardHubUnit.times(ballSpeedMps);
    }

    Rotation2d correctedHeading =
        Rotation2d.fromRadians(Math.atan2(launchVector.getY(), launchVector.getX()));

    ChassisSpeeds driveSpeeds =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            correctedHeading);

    swerve.driveFieldOriented(driveSpeeds);

    double headingErrorDeg =
        Math.abs(robotPose.getRotation().minus(correctedHeading).getDegrees());
    if (headingErrorDeg > 180.0) {
      headingErrorDeg = 360.0 - headingErrorDeg;
    }

    double headingToleranceDeg = currentlySeeingHubTag
        ? Constants.HUB_HEADING_TOLERANCE_DEG_WITH_VISION
        : Constants.HUB_HEADING_TOLERANCE_DEG_NO_VISION;

    boolean drivetrainReady = headingErrorDeg <= headingToleranceDeg;
    boolean shooterReady = shooter.getVelocity().in(RPM) >= shooterRPM * Constants.SHOOTER_READY_FRACTION;

    if (drivetrainReady && shooterReady) {
      indexer.setduty(-1.0);
      hopper.setduty(-1.0);
      CommandScheduler.getInstance().schedule(armOscillateCommand);
    } else {
      indexer.setduty(0.0);
      hopper.setduty(0.0);
    }

    Rotation2d directHubHeading =
        Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));

    SmartDashboard.putBoolean("SOTM/SeeingHubTag", currentlySeeingHubTag);
    SmartDashboard.putNumber("SOTM/LockedTagId", lockedTagId);
    SmartDashboard.putNumber("SOTM/HubCenterX", hubCenterField.getX());
    SmartDashboard.putNumber("SOTM/HubCenterY", hubCenterField.getY());
    SmartDashboard.putNumber("SOTM/PredictedReleaseX", predictedReleasePose.getX());
    SmartDashboard.putNumber("SOTM/PredictedReleaseY", predictedReleasePose.getY());
    SmartDashboard.putNumber("SOTM/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("SOTM/ShooterTargetRPM", shooterRPM);
    SmartDashboard.putNumber("SOTM/NoteSpeedMps", ballSpeedMps);
    SmartDashboard.putNumber("SOTM/CommandedVX", commandedVel.getX());
    SmartDashboard.putNumber("SOTM/CommandedVY", commandedVel.getY());
    SmartDashboard.putNumber("SOTM/MeasuredVX", measuredVel.getX());
    SmartDashboard.putNumber("SOTM/MeasuredVY", measuredVel.getY());
    SmartDashboard.putNumber("SOTM/EstimatedVX", estimatedVel.getX());
    SmartDashboard.putNumber("SOTM/EstimatedVY", estimatedVel.getY());
    SmartDashboard.putNumber("SOTM/DirectHubHeadingDeg", directHubHeading.getDegrees());
    SmartDashboard.putNumber("SOTM/CorrectedHeadingDeg", correctedHeading.getDegrees());
    SmartDashboard.putNumber("SOTM/HeadingErrorDeg", headingErrorDeg);
    SmartDashboard.putNumber("SOTM/LaunchVectorX", launchVector.getX());
    SmartDashboard.putNumber("SOTM/LaunchVectorY", launchVector.getY());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
    indexer.setduty(0.0);
    hopper.setduty(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}