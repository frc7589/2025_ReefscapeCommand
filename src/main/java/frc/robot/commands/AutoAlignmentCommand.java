package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.OpzXboxController;

public class AutoAlignmentCommand extends Command{
    private Swerve m_Swerve;
    private OpzXboxController controller;
    private autoState state;
    private int direction;

    public static enum autoState{
        kIDLE,
        KLeft,
        kRight,
        kFinish
    }

    /**
     * 林樞宇好棒
     * @param direction 0 => 選擇左邊的REEF | 1 => 選擇右邊的REEF
     */
    public AutoAlignmentCommand(int direction, Swerve swerve, OpzXboxController opzXboxController) {
        this.direction = direction;
        this.m_Swerve = swerve;
        this.controller = opzXboxController;
        this.state = autoState.kIDLE;
    }

    @Override
    public void initialize() {
        m_Swerve.resetPID();
        state = direction == 0 ? autoState.KLeft : autoState.kRight; 
        System.out.println("Initializing AutoAlignment: " + state);
    }

    @Override
    public void execute() {
        switch (state) {
            case KLeft:
                m_Swerve.autoalignmentL();
                if(m_Swerve.atSetpoint()) state = autoState.kFinish;
                break;
            case kRight:
                m_Swerve.autoalignmentR();
                if(m_Swerve.atSetpoint()) state = autoState.kFinish;
                break;
            case kFinish:
                System.out.println("isFinshed");
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return state == autoState.kFinish || !controller.a().getAsBoolean();
    }
}
