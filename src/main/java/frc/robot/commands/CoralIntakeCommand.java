package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeCommand extends Command {
    private CoralSubsystem m_intake;
    private Timer m_reverseTimer = new Timer();
    private boolean hasReversed;

    public CoralIntakeCommand(CoralSubsystem m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_reverseTimer.reset();
        hasReversed = false;
    }

    @Override
    public void execute(){
        m_intake.Stop();
        m_reverseTimer.start();
        m_intake.SetReverse(true);

        if(m_reverseTimer.get() > 0.3 && m_reverseTimer.get() < 0.5 && !hasReversed){
            m_intake.ReverseMotor();
            hasReversed = true;
        }

        if(m_reverseTimer.get() > 0.5){
            m_intake.Stop();
            m_reverseTimer.stop();
        }
    }

    @Override
    public boolean isFinished(){
        boolean done = hasReversed && m_reverseTimer.get() >= 0.5;
            if(done){
                m_intake.SetReverse(false);
            }
            return done;
    }
}