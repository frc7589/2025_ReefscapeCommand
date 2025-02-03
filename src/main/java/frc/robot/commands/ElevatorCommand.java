package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{

    //記得加時間的那個

    private ElevatorSubsystem m_elevator;

    private double speed;
    private double waitTime;

    public ElevatorCommand(ElevatorSubsystem elevator, double height, double waitTime) {
        this.speed = speed;
        this.m_elevator = elevator;
        this.waitTime = waitTime;

        addRequirements(m_elevator);
    }

    // 初始化 在呼叫的時候做
    @Override
    public void initialize() {

    }

    // 一直運作
    @Override
    public void execute() {}

    // 結束之後要做什麼
    @Override
    public void end(boolean interrupted) {}

    // 什麼時候結束
    @Override
    public boolean isFinished() {
        return false;
    }
}
