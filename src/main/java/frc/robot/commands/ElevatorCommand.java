package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.OpzXboxController;

public class ElevatorCommand extends Command{
    private Elevator m_Elevator;
    private Timer m_TimeLimit = new Timer();
    private double height;
    private ElevatorHigh level;
    private BooleanSupplier leftY;
    public ElevatorCommand(Elevator elevator, ElevatorHigh level, BooleanSupplier leftY) {
        this.m_Elevator = elevator;
        this.level = level;
        this.leftY = leftY;
        
        addRequirements(m_Elevator);
    }

    public static enum ElevatorHigh{
        kL1("L1"),
        kL2("L2"),
        kL3("L3"),
        kL4("L4");

        private String name;
        private ElevatorHigh(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }
    // Called when the command is initially scheduled.  
    @Override
    public void initialize() {
        m_Elevator.setSetpoint(m_Elevator.getDistance());
        System.out.println("elevator moving");
        m_TimeLimit.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (level) {
            case kL1:
                height = 2;
                break;
            case kL2:
                height = 27.5;
                break;
            case kL3:
                height = 81.4;
                break;
            case kL4:
                height = 168.519;
                break;
            default:
                break;
        }
        m_Elevator.setPosisionCommand(height);
        
        SmartDashboard.putString("Elevator_Level", level.getName());
        //m_Elevator.moving();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("E atsetpoint" + m_Elevator.atSetpoint());
        m_Elevator.resetPID();
        m_TimeLimit.stop();
        m_TimeLimit.reset();
        //m_Elevator.setPosisionCommand(m_Elevator.getDistance());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  m_Elevator.atSetpoint() || leftY.getAsBoolean() || m_TimeLimit.get() > 3;
    }
}
