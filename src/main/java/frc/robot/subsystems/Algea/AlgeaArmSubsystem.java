package frc.robot.subsystems.Algea;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

public class AlgeaArmSubsystem extends SubsystemBase{
   
    private SparkMax m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);

    private RelativeEncoder encoder;
    private DutyCycleEncoder m_EEncoder = new DutyCycleEncoder(0);
    private double offset; //前饋要用 讓他0度的時候水平於地面 要用弧度

    //private ArmFeedforward m_ff = new ArmFeedforward(0, AlgeaConstants.kG, AlgeaConstants.kV, AlgeaConstants.kA);

    //private LinearFilter filter = LinearFilter.movingAverage(5);
    //private double filteredAngle;
    //private double defultposition;

    private PIDController m_PidController = new PIDController(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);
    private PIDController m_fuckyouwholefamily = new PIDController(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);
    private boolean runMotor = false;;

    public AlgeaArmSubsystem() {
        double fuckyou = m_EEncoder.get();

        m_Armmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        encoder = m_Armmotor.getEncoder();

        m_fuckyouwholefamily.setSetpoint(fuckyou);

        m_EEncoder.setInverted(true);
        
        //filter.reset();
        SmartDashboard.putData("fuckyou", m_fuckyouwholefamily);

        SmartDashboard.putNumber("Algea", m_EEncoder.get());


    }

    @Override
    public void periodic() {
       
       // m_Armmotor.set(m_fuckyouwholefamily.calculate(m_EEncoder.get()));

        //filteredAngle = filter.calculate(Math.toRadians(m_EEncoder.get()));

        //double feedforwardVoltage = m_ff.calculate(m_PidController.getSetpoint(), encoder.getVelocity());
        //double pidOutput = m_PidController.calculate(filteredAngle);

        //m_Armmotor.setVoltage(pidOutput + feedforwardVoltage);

        SmartDashboard.putNumber("Algea", m_EEncoder.get());

    }

    public void setArmSpeed(double ArmSpeed) {
        if(m_EEncoder.get() < 0.73) {
            m_Armmotor.set(-ArmSpeed);
        } else {
            m_Armmotor.set(0);
        }
    }

    public double getSetpoint() {
        return m_fuckyouwholefamily.getSetpoint();
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, setpoint, 0.72);
        m_fuckyouwholefamily.setSetpoint(setpoint);
    }

    public void stay() {
        m_Armmotor.set(0.046);
    }
    
    /*public Command setPosisionCommand(double setpoint) {
        return runOnce(() -> m_fuckyouwholefamily.setSetpoint(setpoint));
    }*/
    
}   

