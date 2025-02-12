package frc.robot.subsystems.swerve;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

//import com.fasterxml.jackson.databind.node.BooleanNode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SparkMax Rotor;
    private SparkMax Throttle;

    private RelativeEncoder encoder;
    private CANcoder RotorCancoder;

    private PIDController RotorPID;//沒董

    public SwerveModule(int ThrottleID, int RotorID, int intRotorEncoderID, double RotorEncoderOffsetAngleDeg) {
        Throttle = new SparkMax(ThrottleID, MotorType.kBrushless);

        Rotor = new SparkMax(RotorID, MotorType.kBrushless);

        RotorCancoder = new CANcoder(intRotorEncoderID);

        Throttle.configure(
            new SparkMaxConfig()
                .inverted(SwerveConstants.kThrottleMotorInverted)
                .idleMode(IdleMode.kBrake)
                .apply(new EncoderConfig()
                    .velocityConversionFactor(SwerveConstants.ThrottleVelocityConversionFactor)
                    .positionConversionFactor(SwerveConstants.ThrottlePositionConversionFactor)
                ),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        Rotor.configure(
            new SparkMaxConfig()
                .inverted(SwerveConstants.kRotorMotorInverted)
                .idleMode(IdleMode.kBrake),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        encoder = Throttle.getEncoder();
        //encoder = new SparkRelativeEncoderSim(Throttle);

        
        //將上面設定的東西新增到Cancoder裡面
        RotorCancoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SwerveConstants.kRotorEncoderdiretion)
                .withMagnetOffset(RotorEncoderOffsetAngleDeg/360)
        );

        //根據Constants裡設的常數設置Rotor PID
        RotorPID = new PIDController(
            SwerveConstants.kRotor_P,
            SwerveConstants.kRotor_I,
            SwerveConstants.kRotor_D
        );


        //啟用自動計算從當前值到目標值的最短路徑 180，-180視為同一個點不會嘗試轉一整圈
        RotorPID.enableContinuousInput(-180, 180);
    }
    
    //取得目前swerve模組得狀態(速度、旋轉角度)
    public SwerveModuleState getState() {
        //encoder.setVelocityConversionFactor(SwerveConstants.ThrottleVelocityConversionFactor);
        double ThrottleVelocity = encoder.getVelocity();

        return new SwerveModuleState(
            ThrottleVelocity,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );
    }
    //取得目前swerve模組的狀態(位置、旋轉角度)
    public SwerveModulePosition getPosition() {
        //encoder.setPositionConversionFactor(SwerveConstants.ThrottlePositionConversionFactor);
        double ThrottlePosition = encoder.getPosition();

        return new SwerveModulePosition(
            ThrottlePosition,
            Rotation2d.fromRotations(RotorCancoder.getAbsolutePosition().getValueAsDouble())
        );  
    }
    
    //設定Swerve模組如何運作
    public void setstate(SwerveModuleState state) {
        state.optimize(this.getState().angle);

        //比較目前角度與目標角度利用PID控制器計算出馬達需要輸出多少
        double RotorOutput = RotorPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees());

        Rotor.set(RotorOutput);
        Throttle.set(state.speedMetersPerSecond);
    
    }

    public void setvoltage(Voltage voltage) {
        Throttle.setVoltage(voltage);
    }

    public void setRotorangle() {
        double RotorOutput = RotorPID.calculate(getState().angle.getDegrees(), 90);
        Rotor.set(RotorOutput);
    }

    public double get() {
        return Throttle.get();
    }

    public RelativeEncoder getThrottleEncoder() {
        return Throttle.getAlternateEncoder();
    }
}
