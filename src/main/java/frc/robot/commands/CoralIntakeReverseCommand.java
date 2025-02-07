package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeReverseCommand extends Command {
    private CoralSubsystem m_intake;
    private Timer m_reverseTimer = new Timer();
    private boolean hasReversed;
    private final double stopDistance = 200;

    public CoralIntakeReverseCommand(CoralSubsystem m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_reverseTimer.reset();
        hasReversed = false;
        m_intake.stop();
        m_reverseTimer.start();
    }

    @Override
    public void execute(){
        if(m_reverseTimer.get() > 0.3 && m_reverseTimer.get() < 0.4 ){
            m_intake.reverseMotor();
            hasReversed = true;
            m_intake.setReverse(true);
        }

        if(m_reverseTimer.get() > 0.4 && hasReversed){
            m_intake.stop();
            m_reverseTimer.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setReverse(false);
    }

    @Override
    public boolean isFinished(){
        return m_reverseTimer.get() > 0.4 && m_intake.hasCoral();
    }
}            
