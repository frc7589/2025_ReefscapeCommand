package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeCommand extends Command {
    private CoralSubsystem m_intake;
    private Timer m_Isfinished = new Timer();
    private Timer m_reverseTimer = new Timer();
    private int stage = 0;

    public CoralIntakeCommand(CoralSubsystem m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_Isfinished.start();
        stage = 0;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("stage", stage);
        switch(stage) {
            case 0:
                if (m_intake.hasCoral()) stage++;
                else m_intake.intake();
                break;
            case 1:
                if(m_intake.hasCoral()) m_intake.slowMotor();
                else stage++;
                break;
            case 2:
                if(!m_intake.hasCoral()) m_intake.reverseMotor();
                else {
                    m_reverseTimer.start();
                    stage++;
                }
                break;
            case 3:
                if(m_reverseTimer.get() >= 0.12) stage++;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_reverseTimer.reset();
        m_Isfinished.reset();
        m_intake.updateState();
    }

    @Override
    public boolean isFinished(){
        return stage == 4 || m_Isfinished.get() > 10;
    }
}            
