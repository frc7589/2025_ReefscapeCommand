package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoAlignmentPID extends Command {
    private Swerve m_Swerve;
    private OpzXboxController m_Controller;
    private autoState direction;
    private Pose2d leftPose;
    private Pose2d rightPose;

    public static enum autoState{
        kIDLE,
        KLeft,
        kRight,
        kCoral,
        kFinish
    }

    public AutoAlignmentPID(Swerve swerve, autoState direction, OpzXboxController controller) {
        m_Swerve = swerve;
        m_Controller = controller;
        this.direction = direction;
        addRequirements(m_Swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("oioioioioio");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (direction) {
            case KLeft:
                m_Swerve.autoAlignmentL2(m_Swerve.autoalignmentL());
                break;
            case kRight:
                m_Swerve.autoAlignmentR2(m_Swerve.autoalignmentR());
                break;
            default:
                break;
        }   
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Swerve.resetPID();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_Controller.x().getAsBoolean() || !m_Controller.b().getAsBoolean() ||m_Swerve.isPathfindingAtTarget();
    }
    
}
