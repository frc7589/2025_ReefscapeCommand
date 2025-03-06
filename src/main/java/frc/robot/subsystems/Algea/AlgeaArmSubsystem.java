package frc.robot.subsystems.Algea;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

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
    /*
    private SparkMax m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);

    private RelativeEncoder encoder;
    private DutyCycleEncoder m_EEncoder = new DutyCycleEncoder(1);
    private double offset; //前饋要用 讓他0度的時候水平於地面 要用弧度

    private ArmFeedforward m_ff = new ArmFeedforward(0, AlgeaConstants.kG, AlgeaConstants.kV, AlgeaConstants.kA);

    private LinearFilter filter = LinearFilter.movingAverage(5);
    private double filteredAngle;
    private double defultposition;
    private double targetAngel;

    private PIDController m_PidController = new PIDController(AlgeaConstants.kP, AlgeaConstants.kI, AlgeaConstants.kD);
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

        m_PidController.setTolerance(0.1);

        m_EEncoder.setInverted(true);
        m_PidController.setSetpoint(m_EEncoder.get());
        SmartDashboard.putData("ji", m_PidController);

        SmartDashboard.putNumber("Algea", m_EEncoder.get());


    }

    @Override
    public void periodic() {
        //filteredAngle = filter.calculate(Math.toRadians(m_EEncoder.get()));

        double feedforwardVoltage = m_ff.calculate(Math.toRadians(-30), encoder.getVelocity());
        double pidOutput = m_PidController.calculate(filteredAngle);
        //m_Armmotor.set(m_PidController.calculate(m_EEncoder.get()));

        

        SmartDashboard.putNumber("Algea", m_EEncoder.get());
    }

    public void setArmSpeed(double ArmSpeed) {
        if(m_EEncoder.get() < 0.9) {
            m_Armmotor.set(-ArmSpeed);
        } else {
            m_Armmotor.set(0);
        }
    }

    public double getSetpoint() {
        return m_PidController.getSetpoint();
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, AlgeaConstants.kEncoderOffset, 0.9);
        m_PidController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return m_PidController.atSetpoint();
    }

    
    public void setPosisionCommand(double setpoint) {
        this.setSetpoint(setpoint);
    }

    public void resetPID() {
        m_PidController.reset();
    }
    */
}   

