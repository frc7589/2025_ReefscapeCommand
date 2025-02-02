package frc.robot.subsystems;   

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    private SparkMax m_leftmotor, m_rightmotor;

    public Boolean mode = false;

    private DigitalInput m_sensor = new DigitalInput(0);

    private ColorSensorV3 m_SensorV3 = new ColorSensorV3(null);

    private boolean isReversing;

    public CoralSubsystem() {
        m_leftmotor = new SparkMax(CoralConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightmotor = new SparkMax(CoralConstants.kRightMotorID, MotorType.kBrushless);

        m_rightmotor.configure(
            new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_leftmotor.configure(
            new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void ChangeMode(){
        mode = !mode;
    }

    public void Shoot(){
        if(mode){
            m_leftmotor.set(0.2);
            m_rightmotor.set(0.2);
        }else{
            m_leftmotor.set(0.2);
            m_rightmotor.set(0.4);
        }
    }

    public void Stop(){
        m_leftmotor.set(0);
        m_rightmotor.set(0);
    }

    public boolean hasCoral(){
        return !m_sensor.get();
    }

    public void ReverseMotor(){
        m_leftmotor.set(-0.15);
        m_rightmotor.set(-0.15);
    }

    public void SetReverse(boolean reversing){
        isReversing = reversing;
    }

    public boolean isReversing(){
        return isReversing;
    }
    }
