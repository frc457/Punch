
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE_CONSTANTS;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex intakeMotor = new SparkFlex(INTAKE_CONSTANTS.INTAKE_ID, MotorType.kBrushless);




  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    //setDefaultCommand(indexerMotor.set(0));



    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.smartCurrentLimit(60);
    intakeConfig.idleMode(IdleMode.kCoast);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    public Command set(double dutycycle)
  {
    return run(()->intakeMotor.set(dutycycle));
  }

      public void setduty(double dutyCycle) {
    intakeMotor.set(dutyCycle);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }



  @Override
  public void simulationPeriodic() {

  }
}


