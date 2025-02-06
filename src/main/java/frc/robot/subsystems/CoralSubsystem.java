package frc.robot.subsystems;   

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    private SparkMax m_leftmotor, m_rightmotor;
    private ColorSensorV3 m_ColorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);
    private int proximity = m_ColorSensorV3.getProximity();

    public Boolean mode = false;

    private DigitalInput m_sensor;

    private boolean isReversing;

    public CoralSubsystem() {
        m_leftmotor = new SparkMax(CoralConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightmotor = new SparkMax(CoralConstants.kRightMotorID, MotorType.kBrushless);

        m_sensor = new DigitalInput(CoralConstants.kSensorPortID);

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
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Distance", getDistance());
        
        SmartDashboard.putBoolean("shooterMode", mode);
    } 

    public void changeMode(){
        mode = !mode;
    }

    public void shoot(){
        if(mode){
            m_leftmotor.set(0.2);
            m_rightmotor.set(0.4);
        }else{
            m_leftmotor.set(0.2);
            m_rightmotor.set(0.6);
        }
    }

    public void stop(){
        m_leftmotor.set(0);
        m_rightmotor.set(0);
    }

    public double getDistance(){
        return m_ColorSensorV3.getProximity();
    }

    public boolean hasCoral(){
        return true ? getDistance() > 160 : false;
    }

    public void reverseMotor(){
        m_leftmotor.set(-0.1);
        m_rightmotor.set(-0.1);
    }

    public void setReverse(boolean reversing){
        isReversing = reversing;
    }

    public boolean isReversing(){
        return isReversing;
    }
}
