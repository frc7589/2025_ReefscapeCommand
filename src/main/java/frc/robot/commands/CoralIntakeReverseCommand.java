package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeReverseCommand extends Command {
    private CoralSubsystem m_intake;
    private Timer m_reverseTimer = new Timer();

    public CoralIntakeReverseCommand(CoralSubsystem m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_reverseTimer.reset();
        m_reverseTimer.start();
    }

    @Override
    public void execute(){
        if(m_reverseTimer.get() < 0.2 ){
            m_intake.slowMotor();
            m_intake.setSpin(true);
        }

        if(m_reverseTimer.get() > 0.2 ){
            m_intake.setSpin(false);
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_intake.setSpin(false);
    }

    @Override
    public boolean isFinished(){
        return m_reverseTimer.get() > 0.3 && m_intake.hasCoral();
    }
}            
