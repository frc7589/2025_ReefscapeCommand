package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class AlgeaSubsystem extends SubsystemBase {
    private SparkMax m_Intakemotor, m_Armmotor;
    private DutyCycleEncoder m_ArmEncoder;

    private PIDController pidController;
    private double[] pidarray = {AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD};
        
    private boolean m_enabled;

    public AlgeaSubsystem(){
        m_Intakemotor = new SparkMax(AlgeaConstants.kIntakeMotorID, MotorType.kBrushless);
        m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);

        pidController = new PIDController(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);

        m_ArmEncoder = new DutyCycleEncoder(AlgeaConstants.kEncoderID);

        m_Armmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_Intakemotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic(){
        if(m_enabled){
            m_Armmotor.set(pidController.calculate(m_ArmEncoder.get()));
        }

        SmartDashboard.putData("algea intake pid", pidController);
        SmartDashboard.getNumberArray("algea pid array", pidarray);
        pidController.setPID(pidarray[0], pidarray[1], pidarray[2]);
    }

    public void setState(boolean enable){
        if(enable) {
            m_enabled = true;
            pidController.reset();
        }else{
            m_enabled = false;
        }
    }
    
    public void setSetpoint(double setpoint){
        pidController.setSetpoint(setpoint);
    }

    public double getSetpoint(){
        return pidController.getSetpoint();
    }

    public PIDController getPIDController() {
        return pidController;
    }
}