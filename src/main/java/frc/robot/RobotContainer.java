// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignmentPIDCommand;
import frc.robot.commands.AutoMoveToPoseCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.Stop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algea.AlgeaArmSubsystem;
import frc.robot.subsystems.Algea.AlgeaIntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve m_Swerve = new Swerve();

  private final SendableChooser<Command> m_autoChooser;

  private final Elevator m_Elevator;
  private final AlgeaArmSubsystem m_AlgeaArm;
  private final AlgeaIntakeSubsystem m_AlgeaIntake;
  private final CoralSubsystem m_Shooter;

  private DoubleLogEntry elevatorHighLog;

  private AllianceStationID m_location;
  private Pose2d initialPose;

  //private final AlgeaTestSubsystem m_AlgeaTest = new AlgeaTestSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
 
  private final OpzXboxController m_DriveController = new OpzXboxController(
      OperatorConstants.kDriverControllerPort,
      OperatorConstants.kControllerMinValue);
  private final OpzXboxController m_ActionController = new OpzXboxController(
      OperatorConstants.kActionControllerPort,
      OperatorConstants.kControllerMinValue);
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    elevatorHighLog = new DoubleLogEntry(log, "ElevatorHigh");

    this.m_Shooter = new CoralSubsystem();
    this.m_Elevator = new Elevator();
    this.m_AlgeaArm = new AlgeaArmSubsystem();
    this.m_AlgeaIntake = new AlgeaIntakeSubsystem();
    
    NamedCommands.registerCommand("e1", new ElevatorCommand(m_Elevator, "L1", () -> false));
    NamedCommands.registerCommand("e2", new ElevatorCommand(m_Elevator, "L2", () -> false));
    NamedCommands.registerCommand("e3", new ElevatorCommand(m_Elevator, "L3", () -> false));
    NamedCommands.registerCommand("e4", new ElevatorCommand(m_Elevator, "L4", () -> false));
    NamedCommands.registerCommand("ci", new CoralIntakeCommand(m_Shooter));
    NamedCommands.registerCommand("cs", new AutoShootCommand(m_Shooter));
    NamedCommands.registerCommand("ch", Commands.runOnce(() -> m_Shooter.changeMode(),m_Shooter));
    NamedCommands.registerCommand("AA_L", new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.KLeft, m_DriveController));
    NamedCommands.registerCommand("AA_R", new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kRight, m_DriveController));
    NamedCommands.registerCommand("AA_C", new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kCoral, m_DriveController));
    NamedCommands.registerCommand("Stop", new Stop(m_Swerve));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());

    m_Swerve.setDefaultCommand(Commands.run(
      () -> m_Swerve.drive(
        m_DriveController.getLeftY(),
        m_DriveController.getLeftX(),
        m_DriveController.getRightX()
        ),
        m_Swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    
    m_Elevator.setDefaultCommand(Commands.run(
      () -> {
        m_Elevator.setSetpoint(m_Elevator.getSetpoint() - m_ActionController.getRightY()*1);
      },
      m_Elevator
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    /*
    m_AlgeaArm.setDefaultCommand(Commands.run(() -> {
      m_AlgeaArm.setArmSpeed(m_ActionController.getLeftY()*0.15);
    }, 
    m_AlgeaArm
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    */
    
    m_DriveController.x().or(m_DriveController.b()).whileFalse(Commands.runOnce(
      () -> m_Swerve.setDefaultCommand(Commands.run(
        () -> m_Swerve.drive(
          m_DriveController.getLeftY(),
          m_DriveController.getLeftX(),
          m_DriveController.getRightX()
        ),
        m_Swerve))
    ));
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        
    m_DriveController.leftTrigger().onTrue(m_Swerve.tolowspeed());
    m_DriveController.rightTrigger().onTrue(m_Swerve.tohighSpeed());

    m_DriveController.rightBumper().onTrue(m_Swerve.increaseSpeed());
    m_DriveController.leftBumper().onTrue(m_Swerve.decreaseSpeed());
    
    m_DriveController.x().whileTrue(new AutoAlignmentPIDCommand(m_Swerve, AutoAlignmentPIDCommand.autoState.KLeft).beforeStarting(() -> m_Swerve.removeDefaultCommand()));
    m_DriveController.b().whileTrue(new AutoAlignmentPIDCommand(m_Swerve, AutoAlignmentPIDCommand.autoState.kRight).beforeStarting(() -> m_Swerve.removeDefaultCommand()));
    //m_DriveController.x().onTrue(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.KLeft, m_DriveController).withTimeout(0.01).andThen(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.KLeft, m_DriveController)));
    //m_DriveController.b().onTrue(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kRight, m_DriveController).withTimeout(0.01).andThen(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kRight, m_DriveController)));
    //m_DriveController.y().onTrue(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kCoral, m_DriveController).withTimeout(0.01).andThen(new AutoMoveToPoseCommand(m_Swerve, AutoMoveToPoseCommand.autoState.kCoral, m_DriveController)));
    
    m_DriveController.start().onTrue(m_Swerve.resetHeadingOffset());

    m_ActionController.back().onTrue(new CoralIntakeCommand(m_Shooter));

    m_ActionController.x().whileTrue(Commands.startEnd(
      () -> m_Shooter.shoot(),
      () -> m_Shooter.stop(),
      m_Shooter));

    m_ActionController.b().whileTrue(Commands.startEnd(
      () -> m_Shooter.reverseMotor(),
      () -> m_Shooter.stop(),
      m_Shooter));
      
    m_ActionController.y().whileTrue(Commands.startEnd(
      () -> m_Shooter.slowMotor(),
      () -> m_Shooter.stop(),
      m_Shooter));

    m_ActionController.a().whileTrue(Commands.startEnd(
      () -> m_Shooter.differenrtShoot(),
      () -> m_Shooter.stop(), 
      m_Shooter));
/*
    m_ActionController.start().onTrue(Commands.runOnce(
      () -> m_Shooter.changeMode(),
      m_Shooter));

    m_ActionController.y().whileTrue(Commands.startEnd(
      () -> m_AlgeaIntake.setSpeed(0.4),
      () -> m_AlgeaIntake.setSpeed(0),
      m_AlgeaIntake));

    m_ActionController.a().whileTrue(Commands.startEnd(
      () -> m_AlgeaIntake.setSpeed(-0.4),
      () -> m_AlgeaIntake.setSpeed(0),
      m_AlgeaIntake));
    */
    /*

    m_ActionController.rightTrigger().onTrue(Commands.runOnce(
      () -> elevatorHighLog.append(m_Elevator.getDistance()),
      m_Elevator));
    */
    m_DriveController.povDown().onTrue(Commands.runOnce(
      () -> CommandScheduler.getInstance().cancelAll()));

    /*new Trigger(() -> m_ActionController.getLeftY() != 0).whileFalse(Commands.startEnd(
      () -> m_AlgeaArm.stay(),
      () -> m_AlgeaArm.getDefaultCommand(),
      m_AlgeaArm));*/

    m_ActionController.povUp().onTrue(new ElevatorCommand(m_Elevator, "L4", () -> Math.abs(m_ActionController.getRightY() )> 0));
    m_ActionController.povDown().onTrue(new ElevatorCommand(m_Elevator, "L1", () -> Math.abs(m_ActionController.getRightY() )> 0));
    m_ActionController.povLeft().onTrue(new ElevatorCommand(m_Elevator, "L3", () -> Math.abs(m_ActionController.getRightY() )> 0));
    m_ActionController.povRight().onTrue(new ElevatorCommand(m_Elevator, "L2", () -> Math.abs(m_ActionController.getRightY() )> 0));

  }

  public void robotInit() {
    m_location = DriverStation.getRawAllianceStation();

    switch (m_location) {
      case Blue1:
          initialPose = new Pose2d(7.506, 5.370, Rotation2d.fromDegrees(180));
          break;
      case Blue2:
          initialPose = new Pose2d(7.506, 4.0259, Rotation2d.fromDegrees(180));
          break;
      case Blue3:
          initialPose = new Pose2d(7.506, 1.910, Rotation2d.fromDegrees(180));
          break;
      case Red3:
          initialPose = new Pose2d(10.206944, 5.370, Rotation2d.fromDegrees(0));
          break;
      case Red2:
          initialPose = new Pose2d(10.206944, 4.0259, Rotation2d.fromDegrees(0));
          break;
      case Red1:
          initialPose = new Pose2d(10.206944, 1.910, Rotation2d.fromDegrees(0));
          break;
      default:
          initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
          break;
      }
      m_Swerve.resetAllinace();
      m_Swerve.resetPoseEstimator(m_Swerve.getImuARotation2d(), initialPose);
      m_Swerve.resetReefcoralTargetAngle();
  }

  public void enable() {
    m_Elevator.setSetpoint(m_Elevator.getDistance());
  }

  public void disable() {}

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
