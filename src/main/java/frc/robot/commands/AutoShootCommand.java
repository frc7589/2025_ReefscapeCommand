package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class AutoShootCommand extends Command {
    private CoralSubsystem m_Coral;
    private Timer m_Timer = new Timer();
    private int stage = 0;
    private boolean differenrtShoot;
    public AutoShootCommand(CoralSubsystem coral, boolean differenrtShoot) {
        this.m_Coral = coral;
        this.differenrtShoot = differenrtShoot;
        addRequirements(m_Coral);
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
        stage = 0;
    }

    @Override
    public void execute() {
        switch (stage) {
            case 0:
                if (differenrtShoot) m_Coral.differenrtShoot();
                else m_Coral.shoot();
                if (m_Timer.get() >= 0.4) {
                    stage++;
                }
                break;
            case 1:
            
                m_Coral.stop();
                stage++;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return stage == 2 || m_Timer.get() >= 3;
    }
}
