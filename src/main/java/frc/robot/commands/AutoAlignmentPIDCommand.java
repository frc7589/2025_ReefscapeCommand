package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoMoveToPoseCommand.autoState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoAlignmentPIDCommand extends Command {
    private Pose2d targetPose2d;
    private LEDSubsystem m_Led;
    private Swerve m_driverSubsystem;
    private Command pathfindingCommand;
    private autoState direction;

    public static enum autoState{
        kIDLE,
        KLeft,
        kRight,
        kCoral,
        kFinish
    }

    public AutoAlignmentPIDCommand(LEDSubsystem m_Led, Swerve swerve, autoState direction) {
        this.direction = direction;
        this.m_Led = m_Led;
        this.m_driverSubsystem = swerve;
        addRequirements(m_driverSubsystem);
        System.out.println("con " + this.direction.toString());
    }

    @Override
    public void initialize() {
        System.out.println("start " + this.direction.toString());
        m_Led.setAStage(0);
    }

    @Override
    public void execute() {
        System.out.println("exe " + this.direction.toString());
        switch (direction) {
            case KLeft:
                m_driverSubsystem.autoAlignmentLPID();
                break;
            case kRight:
                m_driverSubsystem.autoAlignmentRPID();
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_driverSubsystem.atSetpoint()) m_Led.setAStage(1);
        System.out.println("end " + this.direction.toString());
        System.out.println("PIDatsetpoint" + m_driverSubsystem.atSetpoint());
        m_driverSubsystem.resetPID();
    }

    @Override
    public boolean isFinished() {
        return m_driverSubsystem.atSetpoint();
    }
}
