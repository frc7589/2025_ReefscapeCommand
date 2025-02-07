// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class SwerveConstants {
    //轉向馬達ID
    public static final int kRightFrontRotorID = 62;
    public static final int kRightRearRotorID = 55;
    public static final int kLeftFrontRotorID = 33;
    public static final int kLeftRearRotorID = 34;

    //動力馬達ID
    public static final int kRightFrontThrottleID = 61;
    public static final int kRightRearThrottleID = 56;
    public static final int kLeftFrontThrottleID = 57;
    public static final int kLeftRearThrottleID = 59;

    //轉向馬達EncoderID
    public static final int kRightFrontEncoderID = 2;
    public static final int kRightRearEncoderID = 3;
    public static final int kLeftFrontEncoderID = 1;
    public static final int kLeftRearEncoderID = 4;

    //轉向馬達encoder以及轉向馬達方向設置
    public static final SensorDirectionValue kRotorEncoderdiretion = SensorDirectionValue.CounterClockwise_Positive;

    //Rotor Encode 偏移量    
    public static final double kLeftFrontRotorEncoderOffset = -41.9238 - 0.1757 + 0.6152;
    public static final double kLeftRearRotorEncoderOffset = -82.793 - 0.08729 + 0.8;
    public static final double kRightFrontRotorEncoderOffset = -39.9023 + 180 + 0.175;
    public static final double kRightRearRotorEncoderOffset = -40.166 + 0.966;

    public static final double kwhatever = 0.60325;//meter

    //swerve kinematics 四輪位置順序(LF,RF,LR,RR)
    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
       new Translation2d(kwhatever/2, kwhatever/2),
       new Translation2d(kwhatever/2, kwhatever/-2),
       new Translation2d(kwhatever/-2, kwhatever/2),
       new Translation2d(kwhatever/-2, kwhatever/-2)
    );

    //Rotor PID constants(還沒搞懂要幹嘛)
    public static final double kRotor_P = 0.008181818182;//0.00585;
    public static final double kRotor_I = 0.0001374545455;//0.00001;
    public static final double kRotor_D = 0;//0.000058;

    public static final double kSpeed = 0.6;

    //Swerve最大速度。加速度
    public static final double kMaxVelocityMeterspersecond = 5.0;
    public static final double kMaxAccelerationMetersPersecond = 3.0;

    //輪徑
    public static final double kwheeldiameterMeter = 0.1016;

    //動力齒輪比
    public static final double kThrottleGearRatio = 6.12;

    //動力速度轉換 通式：(1/齒輪比)*輪徑*PI   (這個也沒有搞懂要幹嘛)
    public static final double ThrottleVelocityConversionFactor =
      (Math.PI * kwheeldiameterMeter) / (60.0 * kThrottleGearRatio);

    public static final double ThrottlePositionConversionFactor =
      (Math.PI * kwheeldiameterMeter) / kThrottleGearRatio;

    public static final double kDefaultSpeed = 0.5;
    
    //電壓最大值
    public static final double kVoltagecompensation = 12.0;

    //最常超時時間
    public static final double kLongtimeoutMs = 100.0;

    public static final boolean kRotorMotorInverted = true;
    public static final boolean kThrottleMotorInverted = true;
    
    
    public static final double kPath_kP = 0;
    public static final double kPath_kI = 0;
    public static final double kPath_kD = 0;

    public static final double kPathZ_kP = 0;
    public static final double kPathZ_kI = 0;
    public static final double kPathZ_kD = 0;

    public static final Translation2d[] kModuleoffsets = {
      new Translation2d(kwhatever/2, kwhatever/2),
      new Translation2d(kwhatever/2, kwhatever/-2),
      new Translation2d(kwhatever/-2, kwhatever/2),
      new Translation2d(kwhatever/-2, kwhatever/-2)
    };

    //質量KG
    public static final double kMass = 0;
    //轉動動量
    public static final double kMOI = calculateMOI(kMass, kModuleoffsets);
    //輪子摩擦係數
    public static final double kWheelCOF = 1; //文檔說不會測，就寫1 :) -> If you are unsure, just use a placeholder value of 1.0.
    //馬達電流限制
    public static final double kdriveCurrentLimit = 0;
    //每個輪子馬德數量
    public static final int kNumMotors = 1;

    public static double calculateMOI(double mass, Translation2d[] moduleOffsets) {
      double moi = 0.0;
      double massPerModule = mass / moduleOffsets.length;
  
      for (Translation2d offset : moduleOffsets) {
          double radius = offset.getNorm();
          moi += massPerModule * radius * radius;
      }
  
      return moi;
  }

    public static final RobotConfig kconfig = new RobotConfig(
        kMass,
        kMOI,
        new ModuleConfig(
          kwheeldiameterMeter,
          kMaxVelocityMeterspersecond,
          kWheelCOF,
          DCMotor.getNEO(kNumMotors),
          kdriveCurrentLimit,
          kNumMotors
        ),
        kModuleoffsets
      );
    }


  public static class ElevatorConstants {
    public static final int kElevatorRMotorID = 47;
    public static final int kElevatorLMotorID = 43;

    public static final int kSWitchID = 0;

    public static final int kEncoderID = 0;

    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMinimumIntegral = 0;
    public static final double kMaximumIntegral = 0.7;

    public static final double kTolerance = 1;
    public static final double kDistancePerRevolution = 0;
  }


  public static class ShooterConstants {
    public static final int kShooterRMotorID = 46;
    public static final int kShooterLMotorID = 45;
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kActionControllerPort = 1;

    //Xbox控制器數值忽略基準值
    public static final double kControllerMinValue = 0.08;
  }


  public static class CoralConstants {
    public static final int kLeftMotorID = 47;
    public static final int kRightMotorID = 45;

    public static final int kSensorPortID = 0;
  }


  public static class AlgeaConstants {
    public static final int kArmMotorID = 30;
    public static final int kIntakeMotorID = 52;

    public static final int kEncoderID = 1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
