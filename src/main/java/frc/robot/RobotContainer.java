// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralIntakeReverseCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgeaSubsystem;
import frc.robot.subsystems.AlgeaTestSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_Swerve = new SwerveDrive();

  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();

  private final CoralSubsystem m_Coral = new CoralSubsystem();
  private final AlgeaSubsystem m_Algea = new AlgeaSubsystem();

  private final AlgeaTestSubsystem m_AlgeaTest = new AlgeaTestSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 

  private final CommandXboxController m_DriveController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_ActionController = 
      new CommandXboxController(OperatorConstants.kActionControllerPort);

  private final XboxController m_Controller = 
      new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_Swerve.setDefaultCommand(Commands.run(
      () -> m_Swerve.drive(
        Math.abs(m_DriveController.getLeftY()) > 0.08 ? m_DriveController.getLeftY() : 0,
        Math.abs(m_DriveController.getLeftX()) > 0.08 ? m_DriveController.getLeftX() : 0,
        Math.abs(m_DriveController.getRightX()) > 0.08 ? -m_DriveController.getRightX() : 0,
        true
      ),
      m_Swerve
    ));
    
    /*m_AlgeaTest.setDefaultCommand(Commands.run(
      () -> {
        m_AlgeaTest.setArmSpeed(
          Math.abs(m_ActionController.getLeftY()) > 0.08 ? m_ActionController.getLeftY() * 0.4 : 0
        );
      },
      m_AlgeaTest
    ));*/

    m_Algea.setDefaultCommand(Commands.run(
      () -> {
        m_Algea.setSetpoint(
          Math.abs(m_DriveController.getRightY()) > 0.08 ? m_DriveController.getRightY() : 0
        );
      }, m_Algea));

    m_Elevator.setDefaultCommand(Commands.run(
      () -> m_Elevator.setOutput(m_Elevator.getPIDOutput()),
      m_Elevator));

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
    new Trigger(m_Coral::hasCoral)
        .onTrue(new CoralIntakeReverseCommand(m_Coral));
        
    m_DriveController.leftTrigger().onTrue(m_Swerve.tolowspeed());
    m_DriveController.rightTrigger().onTrue(m_Swerve.tohighSpeed());

    m_DriveController.rightBumper().onTrue(m_Swerve.increaseSpeed());
    m_DriveController.leftBumper().onTrue(m_Swerve.decreaseSpeed());
    
    m_DriveController.start().onTrue(m_Swerve.switchDriveMode());

    m_ActionController.a().whileTrue(Commands.run(
      () -> m_Elevator.changeSetPosistion(-5),
      m_Elevator));

    m_ActionController.y().whileTrue(Commands.run(
      () -> m_Elevator.changeSetPosistion(5), //5cm
      m_Elevator));

    m_ActionController.start().whileTrue(Commands.runOnce(
      () -> m_Coral.changeMode(),
      m_Coral));

    m_ActionController.x().whileTrue(new ConditionalCommand(
      Commands.startEnd(
        () -> m_Coral.shoot(), 
        () -> m_Coral.stop(),
        m_Coral
      ),
        new InstantCommand(),
        () -> !m_Coral.isReversing())
        );
    
    m_ActionController.povUp().whileTrue(Commands.runOnce(
      () -> m_Elevator.setPosision(0),
      m_Elevator));
    m_ActionController.povRight().whileTrue(Commands.runOnce(
        () -> m_Elevator.setPosision(0),
        m_Elevator));
    m_ActionController.povDown().whileTrue(Commands.runOnce(
        () -> m_Elevator.setPosision(0),
        m_Elevator));
    m_ActionController.povLeft().whileTrue(Commands.runOnce(
        () -> m_Elevator.setPosision(0),
        m_Elevator));

    m_ActionController.b().whileTrue(Commands.startEnd(
      () -> m_Algea.setState(true),
      () -> m_Algea.setState(false),
      m_Algea
    ));

    m_ActionController.rightTrigger().whileTrue(Commands.runOnce(
      () -> m_AlgeaTest.setSuckSpeed(0.4), 
      m_AlgeaTest));

    m_ActionController.leftTrigger().whileTrue(Commands.runOnce(
      () -> m_AlgeaTest.setReleaseSpeed(0.4),
       m_AlgeaTest));
    
  }

}
