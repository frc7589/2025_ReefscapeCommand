package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoMoveToPoseCommand extends Command{
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

    public AutoMoveToPoseCommand(Swerve driverSubsystem, autoState direction, OpzXboxController controller) {
        this.direction = direction;
        this.m_driverSubsystem = driverSubsystem;
        this.controller = controller;
        addRequirements(m_driverSubsystem);
    }
    @Override
    public void initialize() {
            switch (direction) {
                case KLeft:
                    targetPose2d = m_driverSubsystem.autoalignmentL();
                    break;
                case kRight:
                    targetPose2d = m_driverSubsystem.autoalignmentR();
                    break;
                case kCoral:
                    targetPose2d = m_driverSubsystem.getCoralSPose();
                default:
                    break;
            }
        if(targetPose2d != null) {
            pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose2d,
            new PathConstraints(3, 3, 180, 360, 12),
            0.0
            );
            m_driverSubsystem.setAutoalignmentFieldOriented(targetPose2d);
            SmartDashboard.putNumber("targetX", targetPose2d.getX());
            SmartDashboard.putNumber("targetY", targetPose2d.getY());
            SmartDashboard.putNumber("targetAngle", targetPose2d.getRotation().getDegrees());
            pathfindingCommand.schedule();
        } else System.out.println("target null");

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FUCK");
        targetPose2d = null;
        pathfindingCommand.cancel();
        //m_driverSubsystem.setAutoalignmentFieldOriented(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return !(controller.x().getAsBoolean() || controller.b().getAsBoolean()) || AutoBuilder.isPathfindingConfigured();
    }
}
