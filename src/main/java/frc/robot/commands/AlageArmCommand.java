package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algea.AlgeaArmSubsystem;
import frc.robot.subsystems.Algea.AlgeaTestSubsystem;

public class AlageArmCommand extends Command{
    private double height;
    private double init;
    private double controllerOutput;
    private AlgeaArmSubsystem m_Arm;
    public AlageArmCommand(AlgeaArmSubsystem arm, double output, double height) {
        this.m_Arm = arm;
        this.controllerOutput = output;
        this.height = height;
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       m_Arm.setPosisionCommand(height);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Arm.resetPID();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       return m_Arm.atSetpoint();
    }
}
