package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import swervelib.SwerveInputStream;

/**
 * Drives the simulated swerve drive through a {@link SwerveInputStream} aimed with
 * {@link SwerveSubsystem#withHubAim}, like RobotContainer's, with both auto-aim modes, from several places and headings on each alliance, and
 * checks that the robot ends up pointing at its alliance's hub and turns the short way to get there.
 */
class AutoAimSimTest
{
  private static final double LOOP = 0.02;
  private static final double SECONDS = 3.0;
  /**
   * How close counts as aimed, in degrees. The simulated cameras are noisy, and their noise moves the estimated
   * position, and so the bearing to the hub, by a few degrees at times.
   */
  private static final double TOLERANCE = 10.0;

  private static final Translation2d[] POSITIONS = {
      new Translation2d(2.0, 2.0), new Translation2d(2.0, 6.5), new Translation2d(7.0, 1.5),
      new Translation2d(7.0, 6.5), new Translation2d(15.0, 1.0), new Translation2d(14.5, 6.5)};
  private static final double[] HEADINGS = {0, 90, 179, -90, -150, 45};

  private static SwerveSubsystem drivebase;

  @BeforeAll
  static void setup()
  {
    assertTrue(HAL.initialize(500, 0));
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  }

  @AfterAll
  static void teardown()
  {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  private static void enable(AllianceStationID station)
  {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
  }

  /** The heading from the robot to the hub its alliance aims at, in degrees. */
  private static double bearingToHub(boolean red)
  {
    Translation2d hub = drivebase.targets[red ? 1 : 0];
    Translation2d toHub = hub.minus(drivebase.getPose().getTranslation());
    return toHub.getAngle().getDegrees();
  }

  /** Where the simulated robot really is, as opposed to where odometry and vision think it is. */
  private static Pose2d truePose()
  {
    return drivebase.getSwerveDrive().getSimulationDriveTrainPose().orElseThrow();
  }

  /** How far the robot really is from facing the hub, in degrees. */
  private static double trueHeadingError(boolean red)
  {
    Translation2d hub = drivebase.targets[red ? 1 : 0];
    Pose2d pose = truePose();
    return MathUtil.inputModulus(hub.minus(pose.getTranslation()).getAngle().getDegrees()
                                 - pose.getRotation().getDegrees(), -180, 180);
  }

  /** How far the estimated heading is from the true one, in degrees. */
  private static double estimateError()
  {
    return MathUtil.inputModulus(drivebase.getHeading().getDegrees() - truePose().getRotation().getDegrees(),
                                 -180, 180);
  }

  private static double headingError(boolean red)
  {
    return MathUtil.inputModulus(bearingToHub(red) - drivebase.getHeading().getDegrees(), -180, 180);
  }

  private interface Aim
  {
    void start();

    void loop();

    void stop();
  }

  /** Runs one aim from one place and heading; returns a description of what went wrong, or null. */
  private static String run(Aim aim, boolean red, Translation2d position, double heading)
  {
    // Let the last run's spin die down, then place the robot.
    for (int i = 0; i < 75; i++)
    {
      drivebase.drive(new ChassisSpeeds());
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP);
    }
    drivebase.resetOdometry(new Pose2d(position, Rotation2d.fromDegrees(heading)));
    // Let camera frames from before the robot was moved pass, so they don't pull the estimate back.
    for (int i = 0; i < 25; i++)
    {
      drivebase.drive(new ChassisSpeeds());
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP);
    }
    double startError = headingError(red);
    double startHeading = drivebase.getHeading().getDegrees();
    double turned = 0;
    double worstEstimate = Math.abs(estimateError());
    aim.start();
    for (int i = 0; i < SECONDS / LOOP; i++)
    {
      double before = drivebase.getHeading().getDegrees();
      aim.loop();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP);
      turned += MathUtil.inputModulus(drivebase.getHeading().getDegrees() - before, -180, 180);
      worstEstimate = Math.max(worstEstimate, Math.abs(estimateError()));
      // Facing almost straight away from the hub, either way round is as short.
      if (i == 25 && Math.abs(startError) > 20 && Math.abs(startError) < 170
          && Math.signum(turned) != Math.signum(startError))
      {
        aim.stop();
        return String.format("turned the long way: %.0f° off at %.0f°, but turned %.0f° in the first half second "
                             + "(the estimated heading was up to %.0f° from the true one)",
                             startError, startHeading, turned, worstEstimate);
      }
    }
    aim.stop();
    double endError = headingError(red);
    double endTrueError = trueHeadingError(red);
    if (Math.abs(endError) > TOLERANCE)
    {
      return String.format("%.0f° off after %.0f s (really %.0f° off; started %.0f° off at %.0f°, turned %.0f° in all; "
                           + "the estimated heading was up to %.0f° from the true one)",
                           endError, SECONDS, endTrueError, startError, startHeading, turned, worstEstimate);
    }
    return null;
  }

  private static List<String> runAll(Aim aim)
  {
    List<String> failures = new ArrayList<>();
    for (boolean red : new boolean[]{false, true})
    {
      enable(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
      for (Translation2d position : POSITIONS)
      {
        for (double heading : HEADINGS)
        {
          String failure = run(aim, red, position, heading);
          if (failure != null)
          {
            failures.add(String.format("%s at (%.1f, %.1f): %s", red ? "Red" : "Blue", position.getX(),
                                       position.getY(), failure));
          }
        }
      }
    }
    return failures;
  }

  private static void report(String name, List<String> failures)
  {
    int total = 2 * POSITIONS.length * HEADINGS.length;
    System.out.println(name + ": " + (total - failures.size()) + " of " + total + " aimed");
    failures.forEach(f -> System.out.println("  " + f));
  }

  /** A drive stream like RobotContainer's, with the sticks let go, aiming while {@code aiming} is true. */
  private static Aim streamAim(BooleanSupplier aiming, Runnable on, Runnable off)
  {
    Command[] drive = new Command[1];
    return new Aim()
    {
      public void start()
      {
        on.run();
        SwerveInputStream stream = drivebase.withHubAim(
            SwerveInputStream.of(drivebase.getSwerveDrive(), () -> 0.0, () -> 0.0)
                             .withControllerRotationAxis(() -> 0.0)
                             .deadband(0.1)
                             .scaleTranslation(0.8)
                             .allianceRelativeControl(true),
            aiming);
        drive[0] = drivebase.driveFieldOriented(stream);
        CommandScheduler.getInstance().schedule(drive[0]);
      }

      public void loop() {}

      public void stop()
      {
        drive[0].cancel();
        off.run();
        drivebase.drive(new ChassisSpeeds());
      }
    };
  }

  /** R3: aims while the button is held. */
  @Test
  void holdingAimPointsAtTheHub()
  {
    boolean[] held = {false};
    List<String> failures = runAll(streamAim(() -> held[0], () -> held[0] = true, () -> held[0] = false));
    report("Hold R3", failures);
    assertTrue(failures.isEmpty(), failures.size() + " aims failed, see the output");
  }

  /** L3: always point at the target, while the driver isn't touching the sticks. */
  @Test
  void alwaysAimPointsAtTheHub()
  {
    List<String> failures = runAll(streamAim(drivebase::isAlwaysPointingAtTarget,
                                             drivebase::toggleAlwaysPointAtTarget,
                                             drivebase::toggleAlwaysPointAtTarget));
    report("Always aim", failures);
    assertTrue(failures.isEmpty(), failures.size() + " aims failed, see the output");
  }
}
