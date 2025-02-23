package frc.robot.subsystems.Algea;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

public class AlgeaTestSubsystem extends SubsystemBase{
    private SparkMax m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);
    private SparkMax m_Intakemotor = new SparkMax(AlgeaConstants.kIntakeMotorID, MotorType.kBrushless);

    private RelativeEncoder encoder;

    private DutyCycleEncoder m_EEncoder = new DutyCycleEncoder(1);
    private double offset; //前饋要用 讓他0度的時候水平於地面 要用弧度

    private ArmFeedforward m_ff = new ArmFeedforward(0, AlgeaConstants.kG, AlgeaConstants.kV, AlgeaConstants.kA);

    private LinearFilter filter = LinearFilter.movingAverage(5);
    private double filteredAngle;

    private PIDController m_PidController = new PIDController(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);
    private boolean runMotor = false;

    public AlgeaTestSubsystem(){

        m_Armmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        m_Intakemotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(true), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        encoder = m_Armmotor.getEncoder();

        //filter.reset();

        m_PidController.setSetpoint(m_EEncoder.get());

        SmartDashboard.putData(m_PidController);
    }

    @Override
    public void periodic(){   
        
        //TODO 把濾波器跟PID的輸出相加 最後設定馬達的電壓非速度
        
        //filteredAngle = filter.calculate(Math.toRadians(m_EEncoder.get()));

        //double feedforwardVoltage = m_ff.calculate(m_PidController.getSetpoint(), encoder.getVelocity());
        //double pidOutput = m_PidController.calculate(filter.calculate(Math.toRadians(m_EEncoder.get())));

        //m_Armmotor.setVoltage(pidOutput + feedforwardVoltage);


        SmartDashboard.putNumber("Algea", m_EEncoder.get());

        /* 
        if(runMotor) {
            if(m_EEncoder.get() > m_PidController.getSetpoint()) {
                m_Armmotor.set((-m_PidController.calculate(m_EEncoder.get())));
                if(Math.abs(m_PidController.getSetpoint() - m_EEncoder.get()) < 0.02) {
                    m_Armmotor.set(filter.calculate(-m_PidController.calculate(m_EEncoder.get())));
                }
            } else {
                m_Armmotor.set(0);
            }

        } else {
            m_Armmotor.set(0);
        }
        */

        /* 
        if(runMotor) {
            if(m_EEncoder.get() > m_PidController.getSetpoint()) {
                m_Armmotor.setVoltage(pidOutput + feedforwardVoltage);
                
            } else {
                m_Armmotor.setVoltage(0);
            }

        } else {
            m_Armmotor.set(0);
        }
        */
        m_Armmotor.set((-m_PidController.calculate(m_EEncoder.get())));
        
        SmartDashboard.putNumber("A_Output", m_Armmotor.get());
        SmartDashboard.putNumber("A_setpoint", m_PidController.getSetpoint());
    }

    public void setSpeed(double Speed){
        m_Intakemotor.set(Speed);
    }

    public void setArmSpeed(double Armspeed){
        m_Armmotor.set(Armspeed);
    }

    public Command runMotor() {
        return startEnd(() ->  runMotor = true, () -> {
            m_PidController.reset();
            runMotor = false;
        });
    }

    public Command setPosisionCommand(double setpoint) {
        return runOnce(() -> m_PidController.setSetpoint(setpoint));
    }

    public double getEncoder() {
        return m_EEncoder.get();
    }

    public void setSetpoint(double setpoint) {
        m_PidController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return m_PidController.atSetpoint();
    }

    public void resetPID() {
        m_PidController.reset();
    }

    public double getSetpoint() {
        return m_PidController.getSetpoint();
    }

}
