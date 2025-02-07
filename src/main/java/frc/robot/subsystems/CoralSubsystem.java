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

    public Boolean mode = false;

    private DigitalInput m_sensor;

    private boolean isSpining;

    public static enum IntakeState {
        kLoad("已裝載"),
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
                .idleMode(IdleMode.kBrake)
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        m_leftmotor.configure(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        state = this.hasCoral() ? IntakeState.kLoad : IntakeState.kEmpty;
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

    @Override
    public void periodic(){
    SmartDashboard.putBoolean("hasCoral", hasCoral());        
        SmartDashboard.putBoolean("shooterMode", mode);
    } 

    public void changeMode(){
        mode = !mode;
    }

    public void shoot(){
        if(mode){
            this.setRightMode(IdleMode.kBrake);
            m_leftmotor.set(0.1);
            m_rightmotor.set(0.1);
        }else{
            this.setRightMode(IdleMode.kCoast);
            m_leftmotor.set(0.03);
            m_rightmotor.set(0.6);
        }
    }

    public void stop(){
        this.setRightMode(IdleMode.kBrake);
        m_leftmotor.set(0);
        m_rightmotor.set(0);
    }

    public boolean hasCoral(){
        return m_sensor.get();
    }

    public void updateState() {
        this.state = this.hasCoral() ? IntakeState.kLoad : IntakeState.kEmpty;
    }

    public void slowMotor(){
        m_leftmotor.set(0.1);
        m_rightmotor.set(0.1);
    }

    public void intake() {
        m_leftmotor.set(0.3);
        m_rightmotor.set(0.3);
    }

    public void reverseMotor(){
        m_leftmotor.set(-0.2);
        m_rightmotor.set(-0.2);
    }

    public void setSpin(boolean spining){
        isSpining = spining;
    }

    public boolean isSpining(){
        return isSpining;
    }
}
