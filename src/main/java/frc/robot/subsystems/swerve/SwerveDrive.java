package frc.robot.subsystems.swerve;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.VisionSubsystem;

public class SwerveDrive extends SubsystemBase{
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
    
    private SwerveDriveOdometry m_odometry;

    private HashMap<Integer, Rotation2d> targetAngle = new HashMap<>();
            HashMap<Integer, Double> reefTargetAngle = new HashMap<>();

    private double maxspeed = SwerveConstants.kDefaultSpeed;
    //private SysIdRoutine sysid = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(, null))
    private double headingoffset = 0;
    private double min_REEFangle = 360; 
    private double min_CoralDistance = 1000;

    private Integer min_REEFtagID = 0;
    private Integer min_CoralStationtagID = 0;
    
    private Rotation2d inputAngle;

    private boolean fieldOriented = true;
    private boolean doRejectUpdate = false;

    private SwerveDrivePoseEstimator m_poseEstimator;

    private Pose2d inputDistance;

    private Pose3d m_CenterPose;
    private Pose3d m_ReefCenterPose;

    private Field2d m_field = new Field2d();

    private PIDController m_RotationPID;
    private PIDController m_XmotionPID;
    private PIDController m_YmotionPID;

    private PIDController m_autobeinggayPID;

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

    public SwerveDrive() {
        m_CenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().getY())/2,
            0.0f,
            Rotation3d.kZero
        );

        m_ReefCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(13).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(1).get().getY())/2,
            0.0f,
            Rotation3d.kZero
        );

        for(Integer tagID = 17; tagID <= 22; tagID++) {
            var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
            Rotation2d angle = tagPose.getTranslation().minus(m_CenterPose.getTranslation()).toTranslation2d().getAngle();
            targetAngle.put(tagID, angle);
        }

        for(Integer tagID = 1; tagID <= 2 ; tagID++) {
            var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
            double distance = tagpose.getTranslation().getDistance(m_ReefCenterPose.getTranslation());
            reefTargetAngle.put(tagID, distance);
        }

        for(Integer tagID = 12; tagID <= 13 ; tagID++) {
            var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
            double distance = tagpose.getTranslation().getDistance(m_ReefCenterPose.getTranslation());
            reefTargetAngle.put(tagID, distance);
        }

        m_Imu.reset();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kSwerveDriveKinematics,
            Rotation2d.fromDegrees(m_Imu.getYaw()),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.5, 0.5, 0.2),
            VecBuilder.fill(0.7, 0.7, 999999999)
        );
    
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_Imu.getYaw());
        m_odometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveDriveKinematics,
            getRotation2d,
            getModulePositions()
        );

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            (speeds, feedforwards) -> driveChassis(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(
                            SwerveConstants.kPath_kP,
                            SwerveConstants.kPath_kI,
                            SwerveConstants.kPath_kD
                        ),
                    new PIDConstants(
                            SwerveConstants.kPathZ_kP,
                            SwerveConstants.kPathZ_kI,
                            SwerveConstants.kPathZ_kD
                        )
                ),
                new RobotConfig(
                    SwerveConstants.kMass,
                    SwerveConstants.kMOI,
                    new ModuleConfig(
                            SwerveConstants.kWheelRadius,
                            SwerveConstants.kMaxVelocityMeterspersecond,
                            SwerveConstants.kWheelCOF,
                            DCMotor.getNEO(SwerveConstants.kNumMotors),
                            SwerveConstants.kdriveCurrentLimit,
                            SwerveConstants.kNumMotors
                        ),
                        SwerveConstants.klModuleoffsets
                    ),
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this
            );
    }


    @Override
    public void periodic() {
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_Imu.getYaw());
        inputAngle = new Rotation2d(m_Imu.getYaw());
        this.inputDistance = m_poseEstimator.getEstimatedPosition();
        
        m_odometry.update(getRotation2d, getModulePositions());

        SmartDashboard.putNumber("LF", this.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("RF", this.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("LR", this.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("RR", this.getModuleStates()[3].angle.getDegrees());


        SmartDashboard.putNumber("speed", maxspeed);

        SmartDashboard.putNumber("LF_speed", getModuleStates()[0].speedMetersPerSecond);
        SmartDashboard.putNumber("LR_speed", getModuleStates()[2].speedMetersPerSecond);
        SmartDashboard.putNumber("RF_speed", getModuleStates()[1].speedMetersPerSecond);
        SmartDashboard.putNumber("RR_speed", getModuleStates()[3].speedMetersPerSecond);

        //TODO林書宇看過了但我不知道可不可以東西好少
        LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(m_Imu.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(megaTag2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          m_poseEstimator.addVisionMeasurement(
              megaTag2.pose,
              megaTag2.timestampSeconds);
        }

        if (megaTag2.tagCount >= 2) {  // Only trust measurement if we see multiple tags
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                megaTag2.pose,
                megaTag2.timestampSeconds
            );
        }

        m_poseEstimator.update(
            Rotation2d.fromDegrees(m_Imu.getYaw()),
            getModulePositions());

        m_odometry.update(
            getRotation2d,
            getModulePositions());

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putData(m_field);

        targetAngle.forEach((tagID, angle) -> {
            if(inputAngle.minus(angle).getRadians() < this.min_REEFangle) {
                this.min_REEFangle = inputAngle.minus(angle).getRadians();
                this.min_REEFtagID = tagID;
            }
        });

        reefTargetAngle.forEach((tagID, distance) -> {
            if (m_ReefCenterPose.getTranslation().toTranslation2d().getDistance(m_poseEstimator.getEstimatedPosition().getTranslation()) < min_CoralDistance) {
                this.min_CoralDistance = distance;
                this.min_REEFtagID = tagID;
            }
        });

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
    public void 
drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
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
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    -xSpeed*maxspeed, 
                    -ySpeed*maxspeed, 
                    zSpeed*maxspeed, 
                    Rotation2d.fromDegrees(m_Imu.getYaw() - headingoffset)
                )
            );
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed));
            setModulestate(states);
        }
    }
        //TODO 確定是幾號apriltag
        //自動對齊
    public void autoAlignment() {
            m_RotationPID = new PIDController(0, 0, 0);
            m_XmotionPID = new PIDController(0, 0, 0);
            m_YmotionPID = new PIDController(0, 0, 0);
            Pose2d robotpose = m_poseEstimator.getEstimatedPosition();
            Pose2d targetpose = getTargetPose(min_REEFtagID).toPose2d();
            double distance = robotpose.getTranslation().getDistance(targetpose.getTranslation());
            if (distance <= 1.5) {
                drive(
                    m_XmotionPID.calculate(robotpose.getX(), 0),
                    m_YmotionPID.calculate(robotpose.getY(), 0),
                    m_RotationPID.calculate(Rotation2d.fromDegrees(m_Imu.getYaw()).minus(targetAngle.get(this.min_REEFtagID)).getDegrees(), 180)
                );
            }

            
    }
    //TODO 確定是幾號apriltag x y速度未定 不知道會不會被覆蓋掉
    public void autoTurnAround() {
        m_autobeinggayPID = new PIDController(0, 0, 0);
        Pose3d targetPose = getTargetPose(min_CoralStationtagID);
        Pose2d botPose = m_poseEstimator.getEstimatedPosition();
        double distance = botPose.getTranslation().getDistance(targetPose.getTranslation().toTranslation2d());
        if(distance <= 1.5){
            drive(0, 0, m_autobeinggayPID.calculate(Rotation2d.fromDegrees(m_Imu.getYaw()).minus(getRotationToTarget(min_CoralStationtagID)).getDegrees(), 0));
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

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_Imu.getAngle()),
            getModulePositions(),
            pose);
    }

    public ChassisSpeeds getSpeeds () {
        return SwerveConstants.kSwerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveChassis(double xspeed, double ySpeed, double zSpeed) {
        SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xspeed, ySpeed, -zSpeed));
    }

    public void driveChassis(ChassisSpeeds speeds) {
        driveChassis(
            -speeds.vxMetersPerSecond, 
            -speeds.vyMetersPerSecond, 
            -speeds.omegaRadiansPerSecond);
    }
    //將前面返回的state最大速度限制到1再回傳回去給SwerveModuleState
    public void setModulestate (SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        m_LeftFrontModule.setstate(desiredState[0]);
        m_RightFrontModule.setstate(desiredState[1]);
        m_LeftRearModule.setstate(desiredState[2]);
        m_RightRearModule.setstate(desiredState[3]);
    }

    public Pose3d getTargetPose(int ID) {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(ID).get();
    }

    public Rotation2d getRotationToTarget(int ID) {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(ID).get().getRotation().toRotation2d();
    }
}