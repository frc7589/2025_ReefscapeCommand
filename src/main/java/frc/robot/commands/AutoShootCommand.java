package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class AutoShootCommand extends Command {
    private CoralSubsystem mCoral;
    private Timer m_Timer = new Timer();
    private int stage = 0;
    public AutoShootCommand(CoralSubsystem coral) {
        mCoral = coral;
        addRequirements(mCoral);
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
                mCoral.shoot();
                if (m_Timer.get() >= 0.4) {
                    stage++;
                }
                break;
            case 1:
                mCoral.stop();
                stage++;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Timer.stop();
    }

    @Override
    public boolean isFinished() {
        return stage == 2 || m_Timer.get() >= 3;
    }
}
