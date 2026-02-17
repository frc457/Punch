package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsytem;


public class CommandTrain {
    
    private ArmSubsystem m_arm;
    private IndexerSubsystem m_indexer;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
    private TransportSubsytem m_transport;

    public CommandTrain(ArmSubsystem Arm, IndexerSubsystem Indexer, 
                      IntakeSubsystem Intake, ShooterSubsystem Shooter, TransportSubsytem Transport){
        m_arm = Arm;
        m_indexer = Indexer;
        m_intake = Intake;
        m_shooter = Shooter;
        m_transport = Transport;

    }


    public Command shoot(){ 
        return m_shooter.setVelocity(RPM.of(5000)).withTimeout(1)
        .alongWith(m_transport.setVelocity(RPM.of(-1000)).withTimeout(1)) 
        .andThen(
            m_intake.setVelocity(RPM.of(-100)) 
            .alongWith(m_indexer.setVelocity(RPM.of(-100))) 
            .alongWith(m_shooter.setVelocity(RPM.of(5000))) 
            .alongWith(m_transport.setVelocity(RPM.of(-1000))) 
        );
    }


    public Command Intaking(){
        return m_intake.setVelocity(RPM.of(-1000))
        .alongWith(m_indexer.setVelocity(RPM.of(-100)));
    }

    public Command mixer(){
        return m_indexer.setVelocity(RPM.of(1000)).withTimeout(0.5)
        .andThen(m_indexer.setVelocity(RPM.of(-1000)).withTimeout(0.5))
        .andThen(m_indexer.setVelocity(RPM.of(1000)).withTimeout(0.5));
    }
    

    public Command throwup(){
        return m_intake.setVelocity(RPM.of(500)) 
            .alongWith(m_indexer.setVelocity(RPM.of(500)))
            //.alongWith(m_shooter.setVelocity(RPM.of(500))) 
            .alongWith(m_transport.setVelocity(RPM.of(500)));
    }


    //Pathplanner Commands

    public Command timedShoot(){
        return m_shooter.setVelocity(RPM.of(500)).withTimeout(1)
        .alongWith(m_transport.setVelocity(RPM.of(500)).withTimeout(1)) 
        .andThen(
            m_intake.setVelocity(RPM.of(50)) 
            .alongWith(m_indexer.setVelocity(RPM.of(50))) 
            .alongWith(m_shooter.setVelocity(RPM.of(500))) 
            .alongWith(m_transport.setVelocity(RPM.of(500))) 
            .withTimeout(4)
        );
    }


    public Command timedIntaking(){
        return m_intake.setVelocity(RPM.of(100))
        .alongWith(m_indexer.setVelocity(RPM.of(10)))
        .withTimeout(2);
    }

}