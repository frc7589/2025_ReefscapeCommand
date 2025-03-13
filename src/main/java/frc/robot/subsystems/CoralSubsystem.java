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

    private SparkMax m_leftmotor;
    private SparkMax m_rightmotor;

    public Boolean isDifferentSpeed = true;

    private DigitalInput m_sensor;

    private boolean isSpining;

    public static enum IntakeState {
        kLoad("已裝載"),
        kLoading("裝載中"),
        kEmpty("空載");

        private String name;
        private IntakeState(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }

    private IntakeState state;

    public CoralSubsystem() {
        m_leftmotor = new SparkMax(CoralConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightmotor = new SparkMax(CoralConstants.kRightMotorID, MotorType.kBrushless);

        m_sensor = new DigitalInput(CoralConstants.kSensorPortID);

        m_rightmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        m_leftmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setRightMode(IdleMode mode) {
        m_rightmotor.configure(
            new SparkMaxConfig()
                .idleMode(mode)
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }
    public void setLeftMode(IdleMode mode) {
        m_leftmotor.configure(
            new SparkMaxConfig()
                .idleMode(mode)
                .inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("hasCoral", hasCoral());        
        SmartDashboard.putBoolean("shooterMode", isDifferentSpeed);
    } 

    public void changeMode(){
        isDifferentSpeed = !isDifferentSpeed;
    }

    public void shoot(){
        if(isDifferentSpeed){
            this.setRightMode(IdleMode.kBrake);
            this.setLeftMode(IdleMode.kCoast);
            m_rightmotor.set(0.25);
            m_leftmotor.set(0.25);
        }else{ 
            this.setRightMode(IdleMode.kBrake);
            this.setLeftMode(IdleMode.kCoast);
            m_leftmotor.set(0.4);
            m_rightmotor.set(0);
        }
    }

    public void stop(){
        this.setRightMode(IdleMode.kBrake);
        this.setRightMode(IdleMode.kBrake);

        m_leftmotor.set(0);
        m_rightmotor.set(0);
    }

    public void differenrtShoot() {
        this.setRightMode(IdleMode.kBrake);
        this.setLeftMode(IdleMode.kCoast);
        m_leftmotor.set(0.4);
        m_rightmotor.set(0);
    }

    public boolean hasCoral(){
        return m_sensor.get();
    }

    public void slowMotor(){
        m_leftmotor.set(0.15);
        m_rightmotor.set(0.15);
    }

    public void intake() {
        m_leftmotor.set(0.4);
        m_rightmotor.set(0.4);
    }

    public void reverseMotor(){
        m_leftmotor.set(-0.1);
        m_rightmotor.set(-0.1);
    }

    public void setSpin(boolean spining){
        isSpining = spining;
    }

    public boolean isSpining(){
        return isSpining;
    }
}
