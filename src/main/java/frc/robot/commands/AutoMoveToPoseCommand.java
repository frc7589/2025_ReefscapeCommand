package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoMoveToPoseCommand extends Command{
    private Pose2d targetPose2d;
    private Swerve m_driverSubsystem;
    private Command pathfindingCommand;
    private OpzXboxController controller;
    private String direction;
    public AutoMoveToPoseCommand(Swerve driverSubsystem, String direction, OpzXboxController controller) {
        this.direction = direction;
        this.m_driverSubsystem = driverSubsystem;
        this.controller = controller;
        addRequirements(m_driverSubsystem);
    }
    @Override
    public void initialize() {
            switch (direction) {
                case "L":
                    targetPose2d = m_driverSubsystem.autoalignmentL();
                    break;
                case "R":
                    targetPose2d = m_driverSubsystem.autoalignmentR();
                    break;
                default:
                    break;
            }
        if(targetPose2d != null) {
            pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose2d,
            PathConstraints.unlimitedConstraints(SwerveConstants.kVoltagecompensation),
            0.0
            );
        pathfindingCommand.schedule();
        } else System.out.println("target null");

    }

    @Override
    public void end(boolean interrupted) {
        targetPose2d = m_driverSubsystem.getPose();
    }

    @Override
    public boolean isFinished() {
        return !(controller.x().getAsBoolean() || controller.b().getAsBoolean());
    }
}
