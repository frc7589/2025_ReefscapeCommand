package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class AlgeaSubsystem extends SubsystemBase {
    private SparkMax m_Intakemotor, m_Armmotor;
    private final static PIDController m_Pid = new PIDController(0, 0, 0);
    private DutyCycleEncoder m_ArmEncoder;
    private boolean m_enabled;
    private ColorSensorV3 m_ColorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
    private int proximity = m_ColorSensorV3.getProximity();

    public AlgeaSubsystem() {
        m_Intakemotor = new SparkMax(AlgeaConstants.kIntakeMotorID, MotorType.kBrushless);
        m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);

        m_ArmEncoder = new DutyCycleEncoder(0);

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
    public void periodic() {
        if(m_enabled){
            m_Armmotor.set(m_Pid.calculate(m_ArmEncoder.get()));
        }

        SmartDashboard.putData("algea intake pid", m_Pid);

        SmartDashboard.putNumber("proximity", proximity);
    }

    public void setState(boolean enable) {
        if(enable) {
            m_enabled = true;
            m_Pid.reset();
        }else{
            m_enabled = false;
        }
    }
    
    public void setSetpoint(double setpoint) {
        double dposition = m_ArmEncoder.get();
        m_Pid.setSetpoint(dposition - setpoint*0.05);
    }

    public double getSetpoint() {
        return m_Pid.getSetpoint();
    }

    public static PIDController getPIDController() {
        return m_Pid;
    }
}