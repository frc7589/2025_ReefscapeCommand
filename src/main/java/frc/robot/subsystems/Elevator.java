package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private SparkMax m_RMotor = new SparkMax(ElevatorConstants.kElevatorRMotorID, MotorType.kBrushless);
    private SparkMax m_LMotor = new SparkMax(ElevatorConstants.kElevatorLMotorID, MotorType.kBrushless);
    private RelativeEncoder m_Encoder;
    private DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(3);
    private Encoder m_RelEncoder = new Encoder(6, 7);
    private DigitalInput m_limitSwitch = new DigitalInput(5);

    private PIDController pidController = new PIDController(0.08, 0, 0);

    private double offset;
    private double  defultposition;

    private boolean runMotor = false;

    private Timer m_Timer = new Timer();

    //藍6黃7

    public Elevator() {

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

        
        m_AbsEncoder.setInverted(false);

        pidController.setTolerance(1);

        SmartDashboard.putData("Eevator", pidController);
        pidController.setSetpoint(getDistance());
    }

    @Override
    public void periodic() {


        /*if(runMotor) {
            m_RMotor.set(pidController.calculate(this.getPosition()));
        } else {
            m_RMotor.set(0);
        }*/
        m_RMotor.set(pidController.calculate(this.getDistance()));

    
    
        
        //SmartDashboard.putBoolean("touch", limitswitch.get());

        /* 
        if(limitswitch.get()) {
            m_RelEncoder.reset();
        }
        */
        SmartDashboard.putString("Elevator_Level", "base");

        SmartDashboard.putNumber("Elevator_SP", pidController.getSetpoint());
        SmartDashboard.putNumber("Output", m_RMotor.get());
        SmartDashboard.putNumber("ABS_encoder", m_AbsEncoder.get());
        SmartDashboard.putNumber("Elevator_Height", getABSPosition());
        SmartDashboard.putNumber("E_height", this.getDistance());
        SmartDashboard.putBoolean("Elevator_LimitSwitch", m_limitSwitch.get());
        SmartDashboard.putBoolean("runMotor", isEnabled());
        this.elevatorAlert();

        
    }

    public BooleanSupplier pidOnPoint() {
        return () -> pidController.getSetpoint() - m_RelEncoder.getDistance() < 0.5;
    }

    public boolean isEnabled() {    
        return runMotor;
    }

    public boolean atSetpoint() {
        return Math.abs(getSetpoint() - getDistance()) < 3;
    }

    public Command runMotor() {
        return startEnd(() ->  runMotor = true, () -> {
            pidController.reset();
            runMotor = false;
        });
    }

    public void resetPID() {
        pidController.reset();
    }

    public void enable() {
        pidController.reset();
        runMotor = true;
    }

    public void disable() {
        pidController.reset();
        runMotor = false;
    }

    public void moving() {
        //m_RMotor.set(pidController.calculate(getPosition()));
    }

    public double getABSPosition() {
        return m_AbsEncoder.get() - ElevatorConstants.kElevatorAbsOffset;
    }

    public double getDistance() {
    double limitrange = this.getABSPosition() * ElevatorConstants.PositionConversionFactor * 5/4;
        return limitrange;
    }

    
    public double getSetpoint() {
        return pidController.getSetpoint();
    }
    
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, (0.97 - ElevatorConstants.kElevatorAbsOffset) * ElevatorConstants.PositionConversionFactor * 5/4);
        pidController.setSetpoint(setpoint);
    }
    
    public void setPosisionCommand(double setpoint) {
        pidController.reset();
        //setpoint = MathUtil.clamp(setpoint, 0, 1 - ElevatorConstants.kElevatorAbsOffset);
        this.setSetpoint(setpoint);    
    }

    public void elevatorAlert() {
        SmartDashboard.putBoolean("Elevator_Alert", this.getABSPosition() < 0.1);
    }

    
}
