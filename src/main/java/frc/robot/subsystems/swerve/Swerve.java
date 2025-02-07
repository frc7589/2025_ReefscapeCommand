package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase{
    //private final SwerveModule m_LeftFrontModule, m_LeftRearModule, m_RightFrontModule, m_RightRearModule;

    public final SwerveModule m_LeftFrontModule = new SwerveModule(
        SwerveConstants.kLeftFrontThrottleID,
        SwerveConstants.kLeftFrontRotorID,
        SwerveConstants.kLeftFrontEncoderID,
        SwerveConstants.kLeftFrontRotorEncoderOffset
    );

    public final SwerveModule m_LeftRearModule = new SwerveModule(
        SwerveConstants.kLeftRearThrottleID,
        SwerveConstants.kLeftRearRotorID,
        SwerveConstants.kLeftRearEncoderID,
        SwerveConstants.kLeftRearRotorEncoderOffset
    );

    public final SwerveModule m_RightFrontModule = new SwerveModule(
        SwerveConstants.kRightFrontThrottleID,
        SwerveConstants.kRightFrontRotorID,
        SwerveConstants.kRightFrontEncoderID,
        SwerveConstants.kRightFrontRotorEncoderOffset
    );
    

    public final SwerveModule  m_RightRearModule = new SwerveModule(
        SwerveConstants.kRightRearThrottleID,
        SwerveConstants.kRightRearRotorID,
        SwerveConstants.kRightRearEncoderID,
        SwerveConstants.kRightRearRotorEncoderOffset
    );
    
    private final AHRS m_Imu = new AHRS(NavXComType.kMXP_SPI);
    
    private SwerveDriveOdometry odometry;

    private double maxspeed = SwerveConstants.kDefaultSpeed;

    //private SysIdRoutine sysid = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(, null))

    private double headingoffset = 0;

    private boolean fieldOriented = true;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

// Create a new SysId routine for characterizing the drive.
/*private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
                m_LeftFrontModule.setvoltage(voltage);
                m_LeftRearModule.setvoltage(voltage);
                m_RightFrontModule.setvoltage(voltage);
                m_RightRearModule.setvoltage(voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_LeftFrontModule.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_LeftFrontModule.getPosition().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_LeftFrontModule.getState().speedMetersPerSecond, MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));*/

    public Swerve() {
        m_Imu.reset();       
    
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_Imu.getYaw());
        odometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveDriveKinematics,
            getRotation2d,
            getModulePositions()
        );
    }


    @Override
    public void periodic() {
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_Imu.getYaw());
        
        odometry.update(getRotation2d, getModulePositions());

        SmartDashboard.putNumber("LF", this.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("RF", this.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("LR", this.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("RR", this.getModuleStates()[3].angle.getDegrees());


        SmartDashboard.putNumber("speed", maxspeed);

        SmartDashboard.putNumber("LF_speed", getModuleStates()[0].speedMetersPerSecond);
        SmartDashboard.putNumber("LR_speed", getModuleStates()[2].speedMetersPerSecond);
        SmartDashboard.putNumber("RF_speed", getModuleStates()[1].speedMetersPerSecond);
        SmartDashboard.putNumber("RR_speed", getModuleStates()[3].speedMetersPerSecond);

        //m_LeftFrontModule.setRotorangle();
        //m_LeftRearModule.setRotorangle();
        //m_RightFrontModule.setRotorangle();
        //m_RightRearModule.setRotorangle();
    }

    public Command resetHeadingOffset() {
        return runOnce(() -> {
            this.headingoffset = m_Imu.getYaw();
        });
    }

    /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction (X 方向的功率百分比)
     * @param ySpeed percent power in the Y direction (Y 方向的功率百分比)
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (設置機器運動方式) (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_Imu.getYaw());

        if (fieldOriented) {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed, getRotation2d));
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed));
            setModulestate(states);
        }
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        if (fieldOriented) {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed, Rotation2d.fromDegrees(m_Imu.getYaw() - headingoffset)));
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed));
            setModulestate(states);
        }
    }

    public Command switchDriveMode() {
        return runOnce(() -> {
            fieldOriented = !fieldOriented;
        });
    }

    public Command increaseSpeed() {
        return runOnce(() -> {
            if(this.maxspeed > 0.9) return;
            this.maxspeed+=0.1;
        });
    }

    public Command decreaseSpeed() {
        return runOnce(() -> {
            if(this.maxspeed < 0.2) return;
            this.maxspeed-=0.1;
        });
    }

    public Command tohighSpeed() {
        return runOnce(() -> {
            this.maxspeed = 0.8;
        });
    }

    public Command tolowspeed() {
        return runOnce(() -> {
            this.maxspeed = 0.5;
        });
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            m_LeftFrontModule.getState(),
            m_RightFrontModule.getState(),
            m_LeftRearModule.getState(),
            m_RightRearModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            m_LeftFrontModule.getPosition(),
            m_RightFrontModule.getPosition(),
            m_LeftRearModule.getPosition(),
            m_RightRearModule.getPosition()
        };
    }

    //將前面返回的state最大速度限制到1再回傳回去給SwerveModuleState
    public void setModulestate (SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 0.5);
        m_LeftFrontModule.setstate(desiredState[0]);
        m_RightFrontModule.setstate(desiredState[1]);
        m_LeftRearModule.setstate(desiredState[2]);
        m_RightRearModule.setstate(desiredState[3]);
    }

    
}