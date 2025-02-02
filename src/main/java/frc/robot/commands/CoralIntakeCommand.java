package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeCommand extends Command {
    private CoralSubsystem m_intake;
    private Timer m_reverseTimer = new Timer();
    private boolean isFinished;

    public CoralIntakeCommand(CoralSubsystem m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_reverseTimer.reset();
        isFinished = false;
    }

    @Override
    public void execute(){
        m_intake.Stop();
        m_reverseTimer.start();

        if(m_reverseTimer.get() > 0.3 && m_reverseTimer.get() < 0.5){
            m_intake.ReverseMotor();
        }

        if(m_reverseTimer.get() > 0.5){
            m_intake.Stop();
            m_reverseTimer.stop();
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intake.Stop();
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
}