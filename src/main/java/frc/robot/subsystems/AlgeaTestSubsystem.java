package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

public class AlgeaTestSubsystem extends SubsystemBase{
    private SparkMax m_Armmotor, m_Intakemotor;
    

    public AlgeaTestSubsystem(){
        m_Armmotor = new SparkMax(AlgeaConstants.kArmMotorID, MotorType.kBrushless);
        m_Intakemotor = new SparkMax(AlgeaConstants.kIntakeMotorID, MotorType.kBrushless);

        m_Armmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        m_Intakemotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    public void setSuckSpeed(double SuckSpeed){
        m_Intakemotor.set(SuckSpeed);
    }

    public void setReleaseSpeed(double ReleaseSpeed){
        m_Intakemotor.set(ReleaseSpeed);
    }

    public void setArmSpeed(double Armspeed){
        m_Armmotor.set(Armspeed);
    }
}
