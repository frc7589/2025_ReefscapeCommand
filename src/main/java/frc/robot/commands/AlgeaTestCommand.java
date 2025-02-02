package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaTestSubsystem;

public class AlgeaTestCommand extends Command{
    private AlgeaTestSubsystem m_Algea;
    
    public AlgeaTestCommand(AlgeaTestSubsystem m_Algea){
        this.m_Algea = m_Algea;
        addRequirements(m_Algea);
    }

    @Override
    public void initialize(){

    }

    public void execute(){
        
    }
}
