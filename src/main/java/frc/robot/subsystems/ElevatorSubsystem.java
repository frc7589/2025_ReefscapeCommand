package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax m_RMotor = new SparkMax(ElevatorConstants.kElevatorRMotorID, MotorType.kBrushless);
    private SparkMax m_LMotor = new SparkMax(ElevatorConstants.kElevatorLMotorID, MotorType.kBrushless);
    private RelativeEncoder m_Encoder;

    private PIDController pidController = new PIDController(0.4, 0, 0);

    private boolean goingUP = false;
    private boolean goingDOWN = false;
    
    public ElevatorSubsystem() {

        m_RMotor.configure(
            new SparkMaxConfig().
            inverted(false).
            idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        m_LMotor.configure(
            new SparkMaxConfig().
            follow(m_RMotor, true).
            idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        m_Encoder = m_RMotor.getEncoder();
        SmartDashboard.putNumber("setPoint", m_Encoder.getPosition());
        SmartDashboard.putNumber("P", pidController.getP());
        SmartDashboard.putNumber("I", pidController.getI());
        SmartDashboard.putNumber("D", pidController.getD());
    }

    public void setPosision(double setpoint) {
        pidController.setSetpoint(setpoint);
    }


    public void setOutput(double output) {
        SmartDashboard.putNumber("elevatorOutput", output);
        m_RMotor.set(0.5*output);
    }

    public void up() {
        m_RMotor.set(0.7);
    }

    public void down() {
        m_RMotor.set(-0.7);
    }

    public void setSpeed(double tohighSpeed) {
        m_RMotor.set(0.9*tohighSpeed);
    }

    public void stay() {
        m_RMotor.set(0);
        // TODO Stay
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("height", m_Encoder.getPosition()/160);

        // TODO PID控制
        SmartDashboard.putData("PID", pidController);
        SmartDashboard.putNumber("Point", m_Encoder.getPosition());

        pidController.setSetpoint(
            SmartDashboard.getNumber("setPoint", m_Encoder.getPosition())
        );

        pidController.setP(
            SmartDashboard.getNumber("P", 0)
        );

        pidController.setI(
            SmartDashboard.getNumber("I", 0)
        );

        pidController.setD(
            SmartDashboard.getNumber("D", 0)
        );
    }
}
