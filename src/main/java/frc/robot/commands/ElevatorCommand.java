package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command{
    private Elevator m_Elevator;
    private double height;
    private CommandXboxController controller;
    public ElevatorCommand(Elevator elevator, double height, CommandXboxController controller) {
        this.m_Elevator = elevator;
        this.height = height;
        this.controller = controller;
        
        addRequirements(m_Elevator);
    }

        // Called when the command is initially scheduled.  
    @Override
    public void initialize() {
        m_Elevator.setSetpoint(m_Elevator.getDistance());
        System.out.println("elevator moving");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Elevator.setSetpoint(height);
        //m_Elevator.moving();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Elevator.resetPID();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Elevator.atSetpoint() || Math.abs(controller.getRightY()) > 0.1 ? true : false;
    }
}
