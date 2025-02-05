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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax m_RMotor = new SparkMax(ElevatorConstants.kElevatorRMotorID, MotorType.kBrushless);
    private SparkMax m_LMotor = new SparkMax(ElevatorConstants.kElevatorLMotorID, MotorType.kBrushless);
    private DutyCycleEncoder m_Encoder = new DutyCycleEncoder(0);

    private PIDController pidController = new PIDController(0.4, 0, 0);

    private double encoderOffset = 0;
    
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

        SmartDashboard.putNumber("P", pidController.getP());
        SmartDashboard.putNumber("I", pidController.getI());
        SmartDashboard.putNumber("D", pidController.getD());

        pidController.setTolerance(ElevatorConstants.kTolerance);
        pidController.setIntegratorRange(0, 0.7);
        encoderOffset = m_Encoder.get();
    }

    public void changeSetPosistion(double change) {
        pidController.setSetpoint(getSetPosistion() + change);
    }

    public double getPosition() {
        return (m_Encoder.get() - encoderOffset) * ElevatorConstants.kDistancePerRevolution;
    }

    public double getSetPosistion() {
        return pidController.getSetpoint();
    }

    public void setPosision(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public double getPIDOutput() {
        return pidController.calculate(getPosition());
    }

    public void setOutput(double output) {
        SmartDashboard.putNumber("elevatorOutput", output);
        m_RMotor.set(output);
    }


    @Override
    public void periodic() {


        SmartDashboard.putNumber("height", getPosition());
        // TODO PID控制
        SmartDashboard.putData("PID", pidController);

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
