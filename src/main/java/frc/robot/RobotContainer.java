// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignmentCommand;
import frc.robot.commands.CoralIntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgeaSubsystem;
import frc.robot.subsystems.AlgeaTestSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

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

  private final SendableChooser<Command> m_autChooser;

  //private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();

  //private final CoralSubsystem m_Coral = new CoralSubsystem();
  //private final AlgeaSubsystem m_Algea = new AlgeaSubsystem();

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
  
    m_autChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autChooser);
  
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
  //m_DriveController.a().and(() -> m_DriveController.getLeftX() < 0).whileTrue(new AutoAlignmentCommand(0, m_Swerve, m_DriveController));
  

    /*m_Swerve.setDefaultCommand(new ConditionalCommand(
      new ConditionalCommand(
        new AutoAlignmentCommand(
          1,m_Swerve, m_DriveController),
        new AutoAlignmentCommand(
          0,m_Swerve, m_DriveController),
      () -> m_DriveController.getLeftX() > 0),
      Commands.run(
      () -> m_Swerve.drive(
        m_DriveController.getLeftY(), 
        m_DriveController.getLeftX(),
        m_DriveController.getRightX()),
      m_Swerve),
    () -> m_DriveController.a().getAsBoolean())
    );*/
    

    /*
    m_Elevator.setDefaultCommand(Commands.run(
      () -> m_Elevator.setPosision(m_Elevator.getSetpoint() + m_ActionController.getLeftY()*0.1),
      m_Elevator
    ));
     */
    
    /*m_AlgeaTest.setDefaultCommand(Commands.run(
      () -> {
        m_AlgeaTest.setArmSpeed(
          m_ActionController.getLeftY() * 0.4
        );
      },
      m_AlgeaTest
    ));*/

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
    
    //m_DriveController.start().onTrue(m_Swerve.switchDriveMode());
    m_DriveController.start().onTrue(m_Swerve.resetHeadingOffset());

    //m_ActionController.a().whileTrue(new CoralIntakeCommand(m_Coral));

    /* 
    m_ActionController.leftBumper().whileTrue(m_Elevator.runMotor());

    m_ActionController.povUp().onTrue(m_Elevator.setPosisionCommand(180));
    m_ActionController.povDown().onTrue(m_Elevator.setPosisionCommand(20));
    m_ActionController.povLeft().onTrue(m_Elevator.setPosisionCommand(50));
    m_ActionController.povRight().onTrue(m_Elevator.setPosisionCommand(90));
    */

    //m_ActionController.x().whileTrue(Commands.startEnd(() -> m_Coral.shoot(), () -> m_Coral.stop(), m_Coral));
  }

  public void enable() {
    // m_Elevator.enable();
  }

  public void disable() {
    // m_Elevator.disable();
  }

  public Command getAutonomousCommand() {
    //TODO Auto初始化
    
    return m_autChooser.getSelected();
  }
}
