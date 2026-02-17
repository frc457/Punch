
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER_CONSTANTS;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IndexerSubsystem extends SubsystemBase {
    private SparkMax indexerMotor = new SparkMax(INDEXER_CONSTANTS.INDEXER_ID, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(0.06, 0, 0)
  .withSimClosedLoopController(0.06, 0, 0)
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(indexerMotor, DCMotor.getNEO(1), smcConfig);

 private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(4))
  .withMass(Pounds.of(1))
  .withUpperSoftLimit(RPM.of(1000))
  .withTelemetry("IndexerMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
    }   

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
    }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
    }

  /** Creates a new ExampleSubsystem. */
  public IndexerSubsystem() {}

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
    double rpm = shooter.getSpeed().in(RPM);
    SmartDashboard.putNumber("Indexer RPM", rpm);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}


