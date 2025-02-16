package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax m_RMotor = new SparkMax(ElevatorConstants.kElevatorRMotorID, MotorType.kBrushless);
    private SparkMax m_LMotor = new SparkMax(ElevatorConstants.kElevatorLMotorID, MotorType.kBrushless);
    private RelativeEncoder m_Encoder;
    private DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(0);
    private Encoder m_RelEncoder = new Encoder(6, 7);

    //private DigitalInput limitswitch = new DigitalInput(3);

    private PIDController pidController = new PIDController(0.08, 0, 0);

    private double offset;

    private boolean runMotor = false;

    //藍6黃7

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

        //m_Encoder = m_RMotor.getEncoder();
        //m_Encoder.setPosition(0);

        m_RelEncoder.setDistancePerPulse(ElevatorConstants.PositionConversionFactor/8192);
        m_RelEncoder.reset();
        m_RelEncoder.setReverseDirection(true);

        SmartDashboard.putData(pidController);
        pidController.setSetpoint(getSetpoint());

        offset = 0;//ElevatorConstants.PositionConversionFactor*(m_AbsEncoder.get()-ElevatorConstants.kElevatorAbsOffset);


    }

    public double getSetpoint() {
        return m_RelEncoder.getDistance()-offset;
    }

    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public void setOutput(double output) {
        m_RMotor.set(-0.7*output);
        /*if(Math.abs(output) > 0.08){
            m_RMotor.set(0.7*output);
        } else if(Math.abs(output) > 0 && output < 0.08){
            m_RMotor.set(0.03375);
        }*/
    }

    public void stay() {       
        m_RMotor.set(0);
        // TODO Stay
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator_SP", pidController.getSetpoint());                                                                                       

        SmartDashboard.putBoolean("runMotor", isEnabled());

        /*if(runMotor) {
            m_RMotor.set(pidController.calculate(this.getPosition()));
        } else {
            m_RMotor.set(0);
        }*/
        m_RMotor.set(pidController.calculate(this.getSetpoint()));

        SmartDashboard.putNumber("Output", m_RMotor.get())  ;

        SmartDashboard.putNumber("Elevator", getSetpoint());
        SmartDashboard.putNumber("Elevator_ABS", m_AbsEncoder.get()-ElevatorConstants.kElevatorAbsOffset);    
    
        
        //SmartDashboard.putBoolean("touch", limitswitch.get());

        /* 
        if(limitswitch.get()) {
            m_RelEncoder.reset();
        }
        */

    }

    public BooleanSupplier pidOnPoint() {
        return () -> pidController.getSetpoint() - m_RelEncoder.getDistance() < 0.5;
    }

    public Command runMotor() {
        return startEnd(() ->  runMotor = true, () -> {
            pidController.reset();
            runMotor = false;
        });
    }

    public Command setPosisionCommand(double setpoint) {
        return run(() -> {
            pidController.reset();
            pidController.setSetpoint(setpoint);   
        });
               
    }

    public void resetPID() {
        pidController.reset();
    }

    public double getEncoder() {
        return m_RelEncoder.getDistance();
    }

    public void enable() {
        pidController.reset();
        runMotor = true;
    }

    public void disable() {
        pidController.reset();
        runMotor = false;
    }

    public boolean isEnabled() {
        return runMotor;
    }
}