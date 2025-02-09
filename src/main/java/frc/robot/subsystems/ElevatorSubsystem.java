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
    private DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(0);
    private Encoder m_RelEncoder = new Encoder(6, 7);

    private DigitalInput m_limitSwitchBottom = new DigitalInput(3);

    private final PIDController pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

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

        m_RelEncoder.setDistancePerPulse(ElevatorConstants.PositionConversionFactor/8192);
        m_RelEncoder.reset();
        m_RelEncoder.setReverseDirection(true);
        SmartDashboard.putData(pidController);
        pidController.setSetpoint(50);

        offset = 0;
        //ElevatorConstants.PositionConversionFactor*(m_AbsEncoder.get()-ElevatorConstants.kElevatorAbsOffset);
    }

    public double getPosition() {
        return m_RelEncoder.getDistance()-offset;
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public void setPosision(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public Command setPosisionCommand(double setpoint) {
        return runOnce(() -> pidController.setSetpoint(setpoint));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator_SP", pidController.getSetpoint());

        if(runMotor) {
            m_RMotor.set(pidController.calculate(this.getPosition()));
        } else {
            m_RMotor.set(0);
        }

        SmartDashboard.putNumber("Output", m_RMotor.get());

        SmartDashboard.putNumber("Elevator", getPosition());
        
        SmartDashboard.putBoolean("touch", m_limitSwitchBottom.get());

        if(m_limitSwitchBottom.get()) {
            m_RelEncoder.reset();
        }
    }

    public Command runMotor() {
        return startEnd(() ->  runMotor = true, () -> {
            pidController.reset();
            runMotor = false;
        });
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
