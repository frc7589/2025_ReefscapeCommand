package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class Stop extends Command{
    private Swerve m_Swerve;
    private Timer m_Timer = new Timer();
    public Stop (Swerve swerve) {
    this.m_Swerve = swerve;
    addRequirements(m_Swerve);
    }
    
    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
    }

    @Override
    public void execute() {
        m_Swerve.drive(0, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_Timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_Timer.get() > 10;
    }
}
