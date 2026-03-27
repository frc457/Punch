
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HOPPER_CONSTANTS;


public class HopperSubsytem extends SubsystemBase {
  private SparkMax HopperMotor = new SparkMax(HOPPER_CONSTANTS.HOPPER_ID, MotorType.kBrushless);

  /**
   * Set the dutycycle of the transport.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutycycle)
  {
    return run(()->HopperMotor.set(dutycycle));
  }


  public HopperSubsytem() {
     SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.smartCurrentLimit(40);
    indexerConfig.idleMode(IdleMode.kCoast);
    HopperMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    public void setduty(double dutyCycle) {
    HopperMotor.set(dutyCycle);
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}


