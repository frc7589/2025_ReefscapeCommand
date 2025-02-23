package frc.robot.subsystems.Algea;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

public class AlgeaIntakeSubsystem extends SubsystemBase{
    private SparkMax m_Intakemotor = new SparkMax(AlgeaConstants.kIntakeMotorID, MotorType.kBrushless);


    public AlgeaIntakeSubsystem(){

        m_Intakemotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(true), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

    }

    public void setSpeed(double Speed){
        m_Intakemotor.set(Speed);
    }
}
