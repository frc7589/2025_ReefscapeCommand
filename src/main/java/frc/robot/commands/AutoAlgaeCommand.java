package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algea.AlgeaArmSubsystem;
import frc.robot.subsystems.Algea.AlgeaIntakeSubsystem;
import frc.robot.subsystems.swerve.Swerve;

public class AutoAlgaeCommand extends Command {
    /*private Elevator m_Elevator;
    private AlgeaArmSubsystem m_AlgeaArm;
    private AlgeaIntakeSubsystem m_AlgeaIntake;
    private Swerve m_Swerve;
    private AutoAlgaeStage m_Stage;
    private Timer m_SuckingTimer;
    private Timer m_SwerveTimer;
    public AutoAlgaeCommand(Elevator elevator, AlgeaArmSubsystem algeaArm, AlgeaIntakeSubsystem algeaIntake, Swerve swerve) {
        m_Elevator = elevator;
        m_AlgeaArm = algeaArm;
        m_AlgeaIntake = algeaIntake;
        m_Swerve = swerve;
        addRequirements(m_Elevator, m_AlgeaArm, m_AlgeaIntake, m_Swerve);
    }

    public static enum AutoAlgaeStage {
        E_GOINGUP,
        ARM_GOINGUP,
        ARM_SUCKING,
        SWERVE_MOVING,
        STOP,
        IDLE
    }

    @Override
    public void initialize() {
        m_Stage = AutoAlgaeStage.E_GOINGUP;
    }

    @Override
    public void execute() {
        switch (m_Stage) {
            case E_GOINGUP:
                m_Elevator.setPosisionCommand(0);
                if(m_Elevator.atSetpoint()) 
                    m_Stage = AutoAlgaeStage.ARM_GOINGUP;
                break;
        
            case ARM_GOINGUP:
                m_AlgeaArm.setPosisionCommand(0);
                if(m_AlgeaArm.atSetpoint())
                    m_Stage = AutoAlgaeStage.ARM_SUCKING;
                break;
            
            case ARM_SUCKING:
                m_AlgeaIntake.setSpeed(1);
                m_SuckingTimer.start();
                if(m_SuckingTimer.get() > 1) {
                    m_AlgeaIntake.setSpeed(0);
                    m_SuckingTimer.stop();
                    m_Stage = AutoAlgaeStage.SWERVE_MOVING;
                }
                break;
            
            case SWERVE_MOVING:
                m_Swerve.drive(0, 0, 0, false);
                m_SwerveTimer.start();
                if(m_SwerveTimer.get() > 1) {
                    m_Swerve.drive(0, 0, 0, true);
                    m_SwerveTimer.stop();
                    m_Stage = AutoAlgaeStage.STOP;
                }

                break;

            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.resetPID();
        m_AlgeaArm.resetPID();
        m_SwerveTimer.reset();
        m_SwerveTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return m_Stage == AutoAlgaeStage.STOP;
    }*/
    
}
