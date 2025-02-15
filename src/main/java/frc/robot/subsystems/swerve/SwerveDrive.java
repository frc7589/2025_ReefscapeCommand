package frc.robot.subsystems.swerve;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.RelativeEncoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
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
    private HashMap<Integer, Rotation2d> coralTargetAngle = new HashMap<>();

    private double maxspeed = SwerveConstants.kDefaultSpeed;
    //private SysIdRoutine sysid = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(, null))
    private double headingoffset = 0;
    private double min_REEFangle = 360; 
    private double min_CoralAngle = 360;
    private double distance;
    private double coraldistance;

    private Integer min_REEFtagID = 0;
    private Integer min_CoralStationtagID = 0;
    
    private Translation2d inputAngle;

    private boolean fieldOriented = true;
    private boolean doRejectUpdate = false;
    private boolean cameraGotSomething = false;

    private SwerveDrivePoseEstimator m_poseEstimator;
    private LimelightHelpers.PoseEstimate megaTag2;

    private Pose2d inputDistance;
    private Pose2d m_RobotPose;

    private Pose3d m_BlueReefCenterPose;
    private Pose3d m_RedReefCenterPose;
    private Pose3d m_CoralCenterPose;
    

    private RelativeEncoder m_Encoder;

    private Field2d m_field = new Field2d();

    private PIDController m_RotationPID =  new PIDController(0, 0, 0);
    private PIDController m_XmotionPID = new PIDController(0, 0, 0);
    private PIDController m_YmotionPID = new PIDController(0, 0, 0);
    private PIDController m_PID = new PIDController(0, 0, 0);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private StructArrayPublisher<Pose3d> aprilTagPoses = NetworkTableInstance.getDefault().getStructArrayTopic("AprilTags", Pose3d.struct).publish();

    public SwerveDrive() {
        m_Encoder = m_LeftFrontModule.getThrottleEncoder();
        m_BlueReefCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().getY())/2,
            0.0,
            Rotation3d.kZero
        );

        m_RedReefCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(10).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(10).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get().getY())/2,
            0.0,
            Rotation3d.kZero
        );

        m_CoralCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(13).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(12).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(1).get().getY())/2,
            0.0f,
            Rotation3d.kZero
        );
        
        Rotation2d getRotation2d = m_Imu.getRotation2d();
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
        
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            for(Integer tagID = 17; tagID <= 22; tagID++) {
            var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
            Rotation2d angle = tagPose.getTranslation().minus(m_BlueReefCenterPose.getTranslation()).toTranslation2d().getAngle();
            targetAngle.put(tagID, angle);
            }
        } else if(DriverStation.getAlliance().get() == Alliance.Red) {
            for(Integer tagID = 6; tagID <= 11; tagID++) {
            var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
            Rotation2d angle = tagPose.getTranslation().minus(m_RedReefCenterPose.getTranslation()).toTranslation2d().getAngle();
            targetAngle.put(tagID, angle);
            }
        }

        if(DriverStation.getAlliance().get() == Alliance.Blue){
            for(Integer tagID = 12; tagID <= 13 ; tagID++) {
                var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
                Rotation2d angle = tagpose.getTranslation().minus(m_CoralCenterPose.getTranslation()).toTranslation2d().getAngle();
                coralTargetAngle.put(tagID, angle);
            }
        } else if(DriverStation.getAlliance().get() == Alliance.Red) {
            for(Integer tagID = 1; tagID <= 2 ; tagID++) {
                var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(tagID).get();
                Rotation2d angle = tagpose.getTranslation().minus(m_CoralCenterPose.getTranslation()).toTranslation2d().getAngle();
                coralTargetAngle.put(tagID, angle);
            }
        }
        
        m_poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kSwerveDriveKinematics,
        m_Imu.getRotation2d(),
        getModulePositions(),
        Pose2d.kZero,
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
        VecBuilder.fill(0.7, 0.7, 999999999)
        );

        m_Imu.reset();

        m_RotationPID.enableContinuousInput(-180, 180);
    
    }


    @Override
    public void periodic() {
        Translation2d inputAngle = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_BlueReefCenterPose.getTranslation().toTranslation2d());
        Translation2d coralInputAngle = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_CoralCenterPose.getTranslation().toTranslation2d());
        m_RobotPose = m_poseEstimator.getEstimatedPosition();

        
        SmartDashboard.putNumber("minREEFtag", min_REEFtagID);
        SmartDashboard.putBoolean("targetvisible", LimelightHelpers.getTV(""));
        SmartDashboard.putData(m_field);

        m_poseEstimator.update(
            m_Imu.getRotation2d(),
            getModulePositions());


        m_odometry.update(
            m_Imu.getRotation2d(),
            getModulePositions());

        this.updateVisionPose();

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        

        min_REEFangle = 5000;
        distance = m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(m_BlueReefCenterPose.getTranslation().toTranslation2d());

        if(distance <= 1.5) {
            targetAngle.forEach((tagID, angle) -> {
                System.out.println("Robot to Reef distance => " + distance);
                System.out.println(tagID +": " + Math.abs(inputAngle.getAngle().minus(angle).getRadians())); 
                if(Math.abs(inputAngle.getAngle().minus(angle).getRadians()) < this.min_REEFangle) {
                    this.min_REEFangle = Math.abs(inputAngle.getAngle().minus(angle).getRadians());
                    this.min_REEFtagID = tagID;
                }
            });
        }

        min_CoralAngle = 5000;
        coraldistance = m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(m_CoralCenterPose.getTranslation().toTranslation2d());
        
        if(coraldistance <= 1.5) {
            coralTargetAngle.forEach((tagID, angle) -> {
                if(Math.abs(coralInputAngle.getAngle().minus(angle).getRadians() )< this.min_CoralAngle) {
                    this.min_CoralAngle = Math.abs(coralInputAngle.getAngle().minus(angle).getRadians());
                    this.min_CoralStationtagID = tagID;
                }
            });
        }
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
        Rotation2d getRotation2d = m_Imu.getRotation2d();

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
                    Rotation2d.fromDegrees(m_Imu.getAngle() - headingoffset)
                )
            );
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(-xSpeed*maxspeed, -ySpeed*maxspeed, zSpeed*maxspeed));
            setModulestate(states);
        }
    }
        
    public Pose2d getRightReefPose() {
        Pose2d targetPose = getTargetPose(min_REEFtagID).toPose2d();
        Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(targetPose.getRotation());
        Translation2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetPose.getRotation().plus(Rotation2d.fromDegrees(90))));

        Pose2d position = new Pose2d(targetPose.getTranslation().plus(tagToPillar).plus(robotToEage), targetPose.getRotation().unaryMinus());
        return position;
    }

    public Pose2d getLeftReefPose() {
        Pose2d targetPose = getTargetPose(min_REEFtagID).toPose2d();
        Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(targetPose.getRotation());
        Translation2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetPose.getRotation().minus(Rotation2d.fromDegrees(90))));

        Pose2d position = new Pose2d(targetPose.getTranslation().plus(tagToPillar).plus(robotToEage), targetPose.getRotation().unaryMinus());
        return position;
    }

    public Pose2d getCoralSPose() {
        Pose2d tagpose = getTargetPose(min_CoralStationtagID).toPose2d();
        Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(tagpose.getRotation());

        Pose2d CSPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation());
        return CSPosition;
    }

    public Pose2d getBlueProcessorPose() {
        Pose2d tagpose = getTargetPose(3).toPose2d();
        Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(tagpose.getRotation());

        Pose2d BPPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation().unaryMinus());
        return BPPosition;
    }
    
    public Pose2d getRedProcessorPose() {
        Pose2d tagpose = getTargetPose(16).toPose2d();
        Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot, 0.0)).rotateBy(tagpose.getRotation());

        Pose2d BPPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation().unaryMinus());
        return BPPosition;
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

    public void updateVisionPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        cameraGotSomething = LimelightHelpers.getTV("");

        if(cameraGotSomething) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999999));
            m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds);
            m_poseEstimator.resetPosition(m_Imu.getRotation2d(), getModulePositions(), limelightMeasurement.pose);
        }
        SmartDashboard.putBoolean("cameraGotSomething", cameraGotSomething);
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

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(
            m_Imu.getRotation2d(),
            getModulePositions(),
            pose);
    }

    public RelativeEncoder getEncoder() {
        return m_LeftFrontModule.getThrottleEncoder();
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

    public Command resetHeadingOffset() {
        return runOnce(() -> {
            this.headingoffset = m_Imu.getAngle();
        });
    }
}