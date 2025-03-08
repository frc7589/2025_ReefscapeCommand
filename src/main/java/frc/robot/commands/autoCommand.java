package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class autoCommand extends Command{
    private Swerve m_Swerve;
    private Timer m_Timer = new Timer();

    public autoCommand(Swerve swerveSubsystem) {
        this.m_Swerve = swerveSubsystem;
        addRequirements(m_Swerve);
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
    }

    @Override
    public void execute() {
        if (m_Timer.get() < 2) {
            m_Swerve.drive(-0.3, 0, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return m_Timer.get() > 5;
    }
}
