package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.commands.HubShootCommand;
import org.junit.jupiter.api.Test;

/** The shooter speed table and when HubShootCommand feeds. */
class HubShootCommandTest
{

  private static double rpmAt(double meters)
  {
    return HubShootCommand.speedFor(Meters.of(meters)).in(RPM);
  }

  @Test
  void eachAutoSpotGetsItsSpeed()
  {
    for (HubShootCommand.Shot shot : HubShootCommand.SHOTS)
    {
      assertEquals(shot.speed().in(RPM), rpmAt(shot.distance().in(Meters)), 1e-6);
    }
  }

  @Test
  void speedsBetweenShotsAreInterpolated()
  {
    // Halfway from 3.41 m at 2600 RPM to 4.93 m at 3000 RPM.
    assertEquals(2800, rpmAt((3.41 + 4.93) / 2), 1e-6);
  }

  @Test
  void speedsPastTheShotsStayAtTheNearestShot()
  {
    assertEquals(2600, rpmAt(1.0), 1e-6);
    assertEquals(3000, rpmAt(8.0), 1e-6);
  }

  @Test
  void feedsOnlyWhenUpToSpeedAndAimed()
  {
    assertTrue(HubShootCommand.ready(RPM.of(2600), RPM.of(2600), Degrees.of(0)));
    assertTrue(HubShootCommand.ready(RPM.of(2350), RPM.of(2600), Degrees.of(-4)));
    assertFalse(HubShootCommand.ready(RPM.of(2300), RPM.of(2600), Degrees.of(0)), "too slow");
    assertFalse(HubShootCommand.ready(RPM.of(2600), RPM.of(2600), Degrees.of(8)), "not aimed");
    assertFalse(HubShootCommand.ready(RPM.of(2600), RPM.of(2600), Degrees.of(-8)), "not aimed");
  }
}
