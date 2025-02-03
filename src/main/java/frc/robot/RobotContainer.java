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
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem m_Coral = new CoralSubsystem();
  private final AlgeaSubsystem m_Algea = new AlgeaSubsystem();
  private final AlgeaTestSubsystem m_AlgeaTest = new AlgeaTestSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 

  private final CommandXboxController m_DriveController = 
      new CommandXboxController(1);

  private final CommandXboxController m_ActionController = 
      new CommandXboxController(0);

  private final XboxController m_Controller = 
      new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new Trigger(m_Coral::hasCoral)
        .onTrue(new CoralIntakeReverseCommand(m_Coral));

    new Trigger(() -> Math.abs(m_ActionController.getRightY()) >= 0.08)
      .whileTrue(Commands.run(
        () -> m_AlgeaTest.setArmSpeed(m_ActionController.getRightY()*0.4), 
        m_AlgeaTest));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_ActionController.start().whileTrue(Commands.runOnce(
      () -> m_Coral.ChangeMode(),
      m_Coral));

    m_ActionController.y().whileTrue(new ConditionalCommand(
      Commands.startEnd(
        () -> m_Coral.Shoot(), 
        () -> m_Coral.Stop(),
        m_Coral
      ),
        new InstantCommand(),
        () -> !m_Coral.isReversing())
        );
    
    m_ActionController.a().whileTrue(Commands.startEnd(
      () -> m_Algea.setState(true),
      () -> m_Algea.setState(false),
      m_Algea
    ));

    m_ActionController.x().whileTrue(Commands.runOnce(
      () -> m_AlgeaTest.setSuckSpeed(0.4), 
      m_AlgeaTest));

    m_ActionController.b().whileTrue(Commands.runOnce(
      () -> m_AlgeaTest.setReleaseSpeed(0.4),
       m_AlgeaTest));
    
  }

}
