package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoMoveToPoseCommand.autoState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoAlignmentPIDCommand extends Command {
    private Pose2d targetPose2d;
    private Swerve m_driverSubsystem;
    private Command pathfindingCommand;
    private OpzXboxController controller;
    private autoState direction;

    public static enum autoState{
        kIDLE,
        KLeft,
        kRight,
        kCoral,
        kFinish
    }

    public AutoAlignmentPIDCommand(Swerve swerve, autoState direction, OpzXboxController controller) {
        this.direction = direction;
        this.m_driverSubsystem = swerve;
        this.controller = controller;
        addRequirements(m_driverSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        switch (direction) {
            case KLeft:
                m_driverSubsystem.autoalignmentL();
                if(m_driverSubsystem.atSetpoint())
                    direction = autoState.kFinish;
                break;
            case kRight:
                targetPose2d = m_driverSubsystem.autoalignmentR();
                if(m_driverSubsystem.atSetpoint())
                    direction = autoState.kFinish;
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driverSubsystem.resetPID();
    }

    @Override
    public boolean isFinished() {
        return !controller.x().getAsBoolean() || !controller.b().getAsBoolean() || direction == autoState.kFinish;
    }
    
}
