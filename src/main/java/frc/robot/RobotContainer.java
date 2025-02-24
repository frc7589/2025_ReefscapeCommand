// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignmentCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import edu.wpi.first.wpilibj2.command.Command;
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

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
  
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);
    /*
    NamedCommands.registerCommand("Elevator_L1", );
    NamedCommands.registerCommand("Elevator_L2", );
    NamedCommands.registerCommand("Elevator_L3", );
    NamedCommands.registerCommand("Elevator_L4", );
    NamedCommands.registerCommand("CoralIntake", new CoralIntakeCommand(m_Coral));
    NamedCommands.registerCommand("CoralShoot", Commands.startEnd(() -> m_Coral.shoot(), () -> m_Coral.stop(), m_Coral));
    NamedCommands.registerCommand("AutoAlignment_L", new AutoAlignmentCommand(0, m_Swerve, m_DriveController));
    NamedCommands.registerCommand("AutoAlignment_R", new AutoAlignmentCommand(1, m_Swerve, m_DriveController));
    */
    m_Swerve.setDefaultCommand(Commands.run(
      () -> m_Swerve.drive(
        m_DriveController.getLeftY(),
        m_DriveController.getLeftX(),
        m_DriveController.getRightX()
        ),
        m_Swerve));
    m_Elevator.setDefaultCommand(Commands.run(
      () -> {
        m_Elevator.setSetpoint(m_Elevator.getSetpoint() - m_ActionController.getRightY()*0.5);
      },
      m_Elevator
    ));
    
    /*m_AlgeaArm.setDefaultCommand(Commands.run(() -> {
      m_AlgeaArm.setSetpoint(m_AlgeaArm.getSetpoint() - m_ActionController.getLeftY()*0.05);
    }, 
    m_AlgeaArm
    ));
    */
    
    
    m_DriveController.a().whileFalse(Commands.runOnce(
      () -> m_Swerve.setDefaultCommand(Commands.run(
        () -> m_Swerve.drive(
          m_DriveController.getLeftY(),
          m_DriveController.getLeftX(),
          m_DriveController.getRightX()
        ),
        m_Swerve))
    ));
    
    m_DriveController.a().onTrue(Commands.runOnce(
      () -> m_Swerve.removeDefaultCommand(),
      m_Swerve));

    m_DriveController.a().and(() -> m_DriveController.getLeftX() < 0).whileTrue(new AutoAlignmentCommand(0,m_Swerve, m_DriveController));
    m_DriveController.a().and(() -> m_DriveController.getLeftX() > 0).whileTrue(new AutoAlignmentCommand(1,m_Swerve, m_DriveController));
    


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
/*
    m_ActionController.y().whileTrue(Commands.startEnd(
      () -> m_AlgeaIntake.setSpeed(0.3),
      () -> m_AlgeaIntake.setSpeed(0),
      m_AlgeaIntake));

    m_ActionController.a().whileTrue(Commands.startEnd(
      () -> m_AlgeaIntake.setSpeed(-0.3),
      () -> m_AlgeaIntake.setSpeed(0),
      m_AlgeaIntake));
*/
    m_ActionController.start().onTrue(Commands.runOnce(
      () -> m_Shooter.changeMode(),
      m_Shooter));

    m_ActionController.rightTrigger().onTrue(Commands.runOnce(
      () -> elevatorHighLog.append(m_Elevator.getDistance()),
      m_Elevator));

    /*new Trigger(() -> m_ActionController.getLeftY() != 0).whileFalse(Commands.startEnd(
      () -> m_AlgeaArm.stay(),
      () -> m_AlgeaArm.getDefaultCommand(),
      m_AlgeaArm));*/

    m_ActionController.povUp().onTrue(new ElevatorCommand(m_Elevator, 0, m_ActionController));
    m_ActionController.povDown().onTrue(new ElevatorCommand(m_Elevator, 0, m_ActionController));
    m_ActionController.povLeft().onTrue(new ElevatorCommand(m_Elevator, 0, m_ActionController));
    m_ActionController.povRight().onTrue(new ElevatorCommand(m_Elevator, 0, m_ActionController));

  }

  public void enable() {

  }

  public void disable() {

  }

  public Command getAutonomousCommand() {
    //TODO Auto初始化
    
    return m_autoChooser.getSelected();
  }
}
