package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Map;

/**
 * Shoot into the hub from wherever the robot is: the shooter's speed comes from the distance to the hub, and the
 * balls feed once the shooter is up to speed and the robot is aimed. Run it while the drive aims the shooter at the
 * hub (see {@link SwerveSubsystem#withHubAim}), as driver R2 does.
 */
public class HubShootCommand extends Command
{

  /** A shot that went in: how far the middle of the robot was from the middle of the hub, and the shooter's speed. */
  public record Shot(Distance distance, AngularVelocity speed)
  {

    Map.Entry<Double, Double> toEntry()
    {
      return Map.entry(distance.in(Meters), speed.in(RPM));
    }
  }

  /**
   * TUNE HERE. Each auto's shooting spot, at the speed it uses: Left A and Right B shoot at SIDE_TRENCH_VELOCITY,
   * Middle C and D at CORRNER_VELOCITY. Add a shot whenever one goes in from somewhere new: Tracking/hubDistance shows
   * the distance. Between shots the speed is interpolated; past the nearest or farthest shot it stays at that shot's.
   */
  public static final List<Shot> SHOTS = List.of(
      new Shot(Meters.of(3.31), RPM.of(2600)),  // Left A Auto
      new Shot(Meters.of(3.41), RPM.of(2600)),  // Right B Auto
      new Shot(Meters.of(4.93), RPM.of(3000))); // Middle C and D Autos, from the corner

  @SuppressWarnings("unchecked")
  private static final InterpolatingDoubleTreeMap SPEED_BY_METERS =
      InterpolatingDoubleTreeMap.ofEntries(SHOTS.stream().map(Shot::toEntry).toArray(Map.Entry[]::new));

  /** How close the shooter has to be to its speed before feeding, like ShootCommand's 90%. */
  private static final double SPEED_TOLERANCE = 0.10;
  /** How far off the shooter can face the hub and still feed. */
  private static final Angle  AIM_TOLERANCE   = Degrees.of(5);

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem   hopper;
  private final IntakeSubsystem  intake;
  private final SwerveSubsystem  drivebase;
  private final Command          armOscillate;

  public HubShootCommand(ShooterSubsystem shooter,
                         IndexerSubsystem indexer,
                         HopperSubsytem hopper,
                         IntakeSubsystem intake,
                         SwerveSubsystem drivebase,
                         Command armOscillate)
  {
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.intake = intake;
    this.drivebase = drivebase;
    this.armOscillate = armOscillate;
    // Not the drivebase: the driver's drive command keeps running, aiming.
    addRequirements(shooter, indexer, hopper, intake);
  }

  /**
   * The shooter speed for a distance from the hub.
   *
   * @param distance From the middle of the robot to the middle of the hub.
   * @return The speed, interpolated from {@link #SHOTS}.
   */
  public static AngularVelocity speedFor(Distance distance)
  {
    return RPM.of(SPEED_BY_METERS.get(distance.in(Meters)));
  }

  /**
   * Whether to feed: the shooter within {@link #SPEED_TOLERANCE} of its speed, and facing the hub within
   * {@link #AIM_TOLERANCE}.
   *
   * @param speed    The shooter's speed.
   * @param goal     The speed it should be at.
   * @param aimError How far the shooter is from facing the hub.
   * @return true when a ball fed now should go in.
   */
  public static boolean ready(AngularVelocity speed, AngularVelocity goal, Angle aimError)
  {
    boolean atSpeed = speed.in(RPM) >= goal.in(RPM) * (1 - SPEED_TOLERANCE);
    boolean aimed = Math.abs(aimError.in(Degrees)) <= AIM_TOLERANCE.in(Degrees);
    return atSpeed && aimed;
  }

  @Override
  public void execute()
  {
    AngularVelocity goal = speedFor(drivebase.getDistanceToHub());
    shooter.setMechanismVelocitySetpoint(goal);
    boolean ready = ready(shooter.getVelocity(), goal, drivebase.getShooterAimError().getMeasure());
    SmartDashboard.putNumber("Shooter/Goal", goal.in(RPM));
    SmartDashboard.putBoolean("Shooter/Ready", ready);

    if (ready)
    {
      // The same feed as ShootCommand.
      indexer.setduty(-1);
      intake.setduty(-0.8);
      hopper.setduty(-1);
      if (!CommandScheduler.getInstance().isScheduled(armOscillate))
      {
        CommandScheduler.getInstance().schedule(armOscillate);
      }
    } else
    {
      indexer.setduty(0);
      hopper.setduty(0);
      intake.setduty(0);
    }
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    CommandScheduler.getInstance().cancel(armOscillate);
    shooter.setduty(0);
    indexer.setduty(0);
    intake.setduty(0);
    hopper.setduty(0);
    SmartDashboard.putBoolean("Shooter/Ready", false);
  }
}
