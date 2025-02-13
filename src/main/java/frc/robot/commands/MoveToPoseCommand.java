package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class MoveToPoseCommand extends Command{
    private Pose2d targetPose2d;
    private SwerveDrive m_driverSubsystem;
    private Command pathfindingCommand;
    public MoveToPoseCommand(SwerveDrive driverSubsystem, Pose2d targetPose2d) {
        this.targetPose2d = targetPose2d;
        this.m_driverSubsystem = driverSubsystem;
        addRequirements(m_driverSubsystem);
    }
    @Override
    public void initialize() {
        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose2d,
            PathConstraints.unlimitedConstraints(SwerveConstants.kVoltagecompensation),
            0.0
        );
        pathfindingCommand.schedule();
    }
    @Override
    public boolean isFinished() {
        return m_driverSubsystem.isDriverControlling();
    }
}
