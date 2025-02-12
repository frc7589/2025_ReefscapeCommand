package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveCommand extends Command{

    //TODO 記得加時間的那個

    private SwerveDrive m_Swerve;

    private RelativeEncoder m_Encoder;

    private PIDController m_PID;

    private boolean goLeft;
    private boolean goRight;
    private boolean alignment;

    private double speed;
    private double output;
    private double currentEncoder;
    private double lTargetPosition;
    private double rTargetPosition;
    private double waitTime;
    private double targetX;
    private double[] targetpose; 

    public SwerveCommand(SwerveDrive Swerve, boolean Alignment, boolean GoLeft, boolean GoRight) {
        this.speed = speed;
        this.m_Swerve = Swerve;
        this.alignment = Alignment;
        this.goLeft = GoLeft;
        this.goRight = GoRight;
        this.m_Encoder = m_Swerve.getEncoder();
        m_PID = new PIDController(0, 0, 0);

        addRequirements(m_Swerve);
    }

    // 初始化 在呼叫的時候做
    @Override
    public void initialize() {
        m_PID.setPID(
            0,
            0,0);
        if(goLeft) {
            lTargetPosition = m_Encoder.getPosition() + 1;
        }

        if(goRight) {
            rTargetPosition = m_Encoder.getPosition() + 2;
        }
    }

    // 一直運作
    @Override
    public void execute() {
        this.targetpose =  LimelightHelpers.getTargetPose_CameraSpace("");
        targetX = targetpose[1]*100;
        this.currentEncoder = m_Encoder.getPosition();

        if(LimelightHelpers.getTV("") && alignment) {
            output = m_PID.calculate(targetX, 0);
            m_Swerve.drive(0, m_PID.calculate(targetX, 0), 0);
            System.out.println("888888888888888");
            System.out.println(SmartDashboard.getNumber("P", 0));
        }

        if(goLeft) {
            m_Swerve.drive(0, m_PID.calculate(m_Encoder.getPosition(), lTargetPosition), 0);
        }

        if(goRight) {
            m_Swerve.drive(0, m_PID.calculate(m_Encoder.getPosition(), rTargetPosition), 0);
        }


    }

    // 結束之後要做什麼
    @Override
    public void end(boolean interrupted) {}

    // 什麼時候結束
    @Override
    public boolean isFinished() {
        if (goLeft && lTargetPosition - currentEncoder <= 0.1) {
            return true;
        }

        if (goRight && rTargetPosition - currentEncoder <= 0.1) {
            return true;
        }

        if (alignment && targetX <= 2) {
            return true;
        }

        
        return false;
    }
}


