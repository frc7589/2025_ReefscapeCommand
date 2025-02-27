package frc.robot.subsystems.swerve;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;

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
    
    private final AHRS m_AHRS = new AHRS(NavXComType.kMXP_SPI);

    private HashMap<Integer, Rotation2d> targetAngle = new HashMap<>();
    private HashMap<Integer, Rotation2d> coralTargetAngle = new HashMap<>();

    private double headingoffset = 0;
    private double min_REEFangle = 360; 
    private double min_CoralAngle = 360;
    private double distance;
    private double coraldistance;
    private double maxSpeedRatio = SwerveConstants.kDefaultSpeed;
    private double headingOffset = 0;
    
    
    private Integer min_REEFtagID = 0;
    private Integer min_CoralStationtagID = 0;

    private Alliance m_alliance;

    private SwerveDrivePoseEstimator m_poseEstimator;
    private SwerveDriveOdometry odometry;

    private LimelightHelpers.PoseEstimate limelightMeasurement;


    private boolean fieldOriented = true;
    private boolean cameraGotSomething = false;

    public static boolean ffControl = true;

    private Pose3d m_BlueReefCenterPose;
    private Pose3d m_RedReefCenterPose;
    private Pose3d m_CoralCenterPose;

    private Pose2d m_RobotPose;

    private PIDController m_RotationPID =  new PIDController(0, 0, 0);
    private PIDController m_XmotionPID = new PIDController(0, 0, 0);
    private PIDController m_YmotionPID = new PIDController(0, 0, 0);

    private LinearFilter m_Filter = LinearFilter.singlePoleIIR(distance, coraldistance);

    private Translation2d inputAngle;

    private Field2d m_field = new Field2d();

    RobotConfig config;{
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        config = SwerveConstants.kconfig;
        e.printStackTrace();
        }
}
    
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
        m_RotationPID.setTolerance(3);
        

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            m_alliance = alliance.get();
        }
        m_AHRS.reset();       

        SmartDashboard.putBoolean("swerve_ffControlled", ffControl);
        SmartDashboard.putData(m_field);
    
        Rotation2d getRotation2d = Rotation2d.fromDegrees(m_AHRS.getYaw());
        odometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveDriveKinematics,
            getRotation2d,
            getModulePositions()
        );

         m_BlueReefCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(18).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(21).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(18).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(21).get().getY())/2,
            0.0,
            Rotation3d.kZero
        );

        m_RedReefCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(10).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(7).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(10).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(7).get().getY())/2,
            0.0,
            Rotation3d.kZero
        );

        m_CoralCenterPose = new Pose3d(
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(13).get().getX() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(12).get().getX())/2,
            (AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(12).get().getY() + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(1).get().getY())/2,
            0.0f,
            Rotation3d.kZero
        );

        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue){
            for(Integer tagID = 17; tagID <= 22; tagID++) {
                System.out.println("inputTagID =>" + tagID);
            var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get();
            Rotation2d angle = tagPose.getTranslation().minus(m_BlueReefCenterPose.getTranslation()).toTranslation2d().getAngle();
            targetAngle.put(tagID, angle);
            }
        } else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            for(Integer tagID = 6; tagID <= 11; tagID++) {
                System.out.println("inputTagID =>" + tagID);
            var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get();
            Rotation2d angle = tagPose.getTranslation().minus(m_RedReefCenterPose.getTranslation()).toTranslation2d().getAngle();
            targetAngle.put(tagID, angle);
            }
        }

        if(m_alliance == Alliance.Blue){
            for(Integer tagID = 12; tagID <= 13 ; tagID++) {
                var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get();
                Rotation2d angle = tagpose.getTranslation().minus(m_CoralCenterPose.getTranslation()).toTranslation2d().getAngle();
                coralTargetAngle.put(tagID, angle);
            }
        } else if(m_alliance == Alliance.Red) {
            for(Integer tagID = 1; tagID <= 2 ; tagID++) {
                var tagpose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get();
                Rotation2d angle = tagpose.getTranslation().minus(m_CoralCenterPose.getTranslation()).toTranslation2d().getAngle();
                coralTargetAngle.put(tagID, angle);
            }
        }
        
        m_poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kSwerveDriveKinematics,
        getImuARotation2d(),//m_Imu.getRotation2d().unaryMinus(),
        getModulePositions(),
        Pose2d.kZero,//initialPose,
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
        VecBuilder.fill(0.7, 0.7, 999999999)
        );

        m_AHRS.reset();

        m_RotationPID.enableContinuousInput(-180, 180);

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::driveChassis,
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
                ),config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }
    
    
    
    @Override
    public void periodic() {        
        if(m_alliance != null) {
            if (m_alliance == Alliance.Blue) {
                inputAngle = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_BlueReefCenterPose.getTranslation().toTranslation2d());
            }
            else 
                inputAngle = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_RedReefCenterPose.getTranslation().toTranslation2d());

        }
        Translation2d coralInputAngle = m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_CoralCenterPose.getTranslation().toTranslation2d());
        Rotation2d getRotation2d = getImuARotation2d();//m_Imu.getRotation2d().unaryMinus();
        m_RobotPose = m_poseEstimator.getEstimatedPosition();
        //m_RobotPose.getRotation().getDegrees() = m_Filter.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        odometry.update(getRotation2d, getModulePositions());

        SmartDashboard.putNumber("LF", this.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("RF", this.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("LR", this.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("RR", this.getModuleStates()[3].angle.getDegrees());


        SmartDashboard.putNumber("speed", maxSpeedRatio);

        SmartDashboard.putNumber("LF_speed", getModuleStates()[0].speedMetersPerSecond);
        SmartDashboard.putNumber("LR_speed", getModuleStates()[2].speedMetersPerSecond);
        SmartDashboard.putNumber("RF_speed", getModuleStates()[1].speedMetersPerSecond);
        SmartDashboard.putNumber("RR_speed", getModuleStates()[3].speedMetersPerSecond);

        SmartDashboard.putData("X_PID", m_XmotionPID);
        SmartDashboard.putData("Y_PID", m_YmotionPID);
        SmartDashboard.putData("R_PID", m_RotationPID);
        SmartDashboard.putNumber("minREEFID", min_REEFtagID);
        SmartDashboard.putNumber("X Position", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y Position", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Ange", m_AHRS.getRotation2d().getDegrees());
        SmartDashboard.putNumber("InvertedAnge", m_AHRS.getRotation2d().unaryMinus().getDegrees());

        //HttpCamera limelight = new HttpCamera("limelight", "http://10.75.89.200:5800");
        //CameraServer.startAutomaticCapture(limelight);
        //limelight.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
     
        Swerve.ffControl = SmartDashboard.getBoolean("swerve_ffControlled", ffControl);

        m_poseEstimator.update(
            getImuARotation2d(),
            getModulePositions());


        odometry.update(
            getImuARotation2d(),
            getModulePositions());

        this.updateVisionPose();

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        
        if(m_alliance != null) {
            if (m_alliance == Alliance.Blue) {
                distance = m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(m_BlueReefCenterPose.getTranslation().toTranslation2d());
            }
            else 
                distance = m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(m_RedReefCenterPose.getTranslation().toTranslation2d());
        }


        min_REEFangle = 5000;
        if(distance <= 3) {
            targetAngle.forEach((tagID, angle) -> {
                //System.out.println("Robot to Reef distance => " + distance);
                //System.out.println(tagID +": " + Math.abs(inputAngle.getAngle().minus(angle).getRadians())); 
                if(Math.abs(inputAngle.getAngle().minus(angle).getRadians()) < this.min_REEFangle) {
                    this.min_REEFangle = Math.abs(inputAngle.getAngle().minus(angle).getRadians());
                    this.min_REEFtagID = tagID;
                }
            });
        }

        min_CoralAngle = 5000;
        coraldistance = m_poseEstimator.getEstimatedPosition().getTranslation().getDistance(m_CoralCenterPose.getTranslation().toTranslation2d());
        
 
            coralTargetAngle.forEach((tagID, angle) -> {
                if(Math.abs(coralInputAngle.getAngle().minus(angle).getRadians() )< this.min_CoralAngle) {
                    this.min_CoralAngle = Math.abs(coralInputAngle.getAngle().minus(angle).getRadians());
                    this.min_CoralStationtagID = tagID;
                }
            });
        
    }

    public Rotation2d getImuARotation2d() {
        if (m_alliance == Alliance.Blue) return m_AHRS.getRotation2d().unaryMinus().plus(Rotation2d.fromDegrees(180));
        return m_AHRS.getRotation2d().unaryMinus();
    }

    public void resetPoseEstimator(Rotation2d rotation, Pose2d pose) {
        m_poseEstimator.resetPosition(rotation, getModulePositions(), pose);
    }

    public void reserImu() {
        m_AHRS.reset();
    }

    public void resetAllinace() {
        m_alliance = DriverStation.getAlliance().get();
    }
    
    public Command resetHeadingOffset() {
        return runOnce(() -> {
            this.headingOffset = m_AHRS.getYaw();
        });
    }

    public void setHeadingAngle(double heading) {
        this.headingOffset = heading;
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
        if(Swerve.ffControl) {
            xSpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
            ySpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
            zSpeed *= SwerveConstants.kMaxAngularVelocityRadPerSecond;

            SmartDashboard.putNumber("x_speed_set", xSpeed);
            SmartDashboard.putNumber("y_speed_set",  ySpeed);
        }
        if (fieldOriented) {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, Rotation2d.fromDegrees(m_AHRS.getYaw() - this.headingOffset)));
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
            setModulestate(states);
        }
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        if(Swerve.ffControl) {
            xSpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
            ySpeed *= SwerveConstants.kMaxVelocityMetersPerSecond;
            zSpeed *= SwerveConstants.kMaxAngularVelocityRadPerSecond;

            SmartDashboard.putNumber("x_speed_set", xSpeed);
            SmartDashboard.putNumber("y_speed_set",  ySpeed);
        }

        if (fieldOriented) {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, Rotation2d.fromDegrees(m_AHRS.getYaw() - this.headingOffset)));
            setModulestate(states);
        } else {
            SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
            setModulestate(states);
        }
    }

    public Pose2d autoalignmentL() {
        Pose3d targetPose = getTargetPose(min_REEFtagID);
        Pose2d position;
        
        if(targetPose != null && getAlliance() != null) {
            Pose2d targetPose2d = targetPose.toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2 + 0.15, 0.0)).rotateBy(targetPose2d.getRotation());
            Translation2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetPose2d.getRotation().minus(Rotation2d.fromDegrees(90))));
            position = new Pose2d(targetPose2d.getTranslation().plus(tagToPillar).plus(robotToEage), targetPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
            System.out.println("target x => " + position.getX());
            System.out.println("target Y => " + position.getY());

            return position;
            /*this.setHeadingAngle(0);

            m_XmotionPID.setSetpoint(position.getX());
            m_YmotionPID.setSetpoint(position.getY());
            //if(getAlliance() == Alliance.Blue) {
                this.drive(
                    m_XmotionPID.calculate(m_RobotPose.getX()),// * (m_RobotPose.getX() - position.getX()) < 0 ? -1 : 1,
                    m_YmotionPID.calculate(m_RobotPose.getY()),// * (m_RobotPose.getY() - position.getY()) < 0 ? -1 : 1, 
                    0,
                    true);//m_Filter.calculate(m_RotationPID.calculate(m_Imu.getYaw(), position.getRotation().getDegrees())));

                    SmartDashboard.putNumber("XPID_output", m_XmotionPID.calculate(m_RobotPose.getX(), position.getX()));// * (m_RobotPose.getX() - position.getX()) < 0 ? -1 : 1);
                    SmartDashboard.putNumber("YPID_output", m_XmotionPID.calculate(m_RobotPose.getY(), position.getY()));// * (m_RobotPose.getX() - position.getX()) < 0 ? -1 : 1);
                    SmartDashboard.putNumber("check this", m_RobotPose.getX());   
                    SmartDashboard.putNumber("X", robotToEage.getX());
                    if(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(min_REEFtagID).isPresent()){
                        System.out.println(min_REEFtagID +" => " + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(min_REEFtagID).get().getX());
                        System.out.println(min_REEFtagID +" => " + AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(min_REEFtagID).get().getRotation().toRotation2d().getDegrees());
                        System.out.println("tag x position =>" + position.getX() + "\nXPID_setpoint =>" + m_XmotionPID.getSetpoint());
                        System.out.println("tag y position =>" + position.getY() + "\nYPID_setpoint =>" + m_YmotionPID.getSetpoint());
                    }*/

                    
           // } else {
                /*this.drive(
                    -m_XmotionPID.calculate(m_RobotPose.getX(), position.getX()), 
                    -m_YmotionPID.calculate(m_RobotPose.getY(), position.getY()), 
                    0,
                    false);//m_Filter.calculate(m_RotationPID.calculate(m_Imu.getYaw(), position.getRotation().getDegrees())));
                    SmartDashboard.putNumber("output", m_XmotionPID.calculate(m_XmotionPID.calculate(m_RobotPose.getX(), position.getX())));
                    SmartDashboard.putNumber("check this", m_RobotPose.getX());*/           
         //   }
        }
         
       return m_RobotPose;
    }

    public Pose2d autoalignmentR() {
        Pose3d targetPose = getTargetPose(min_REEFtagID);
        Pose2d targetpose2d;
        if(targetPose != null) {
            targetpose2d = targetPose.toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2 + 0.15, 0.0)).rotateBy(targetpose2d.getRotation());
            Translation2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetpose2d.getRotation().plus(Rotation2d.fromDegrees(90))));

            Pose2d position = new Pose2d(targetpose2d. getTranslation().plus(tagToPillar).plus(robotToEage), targetpose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
            System.out.println("target x => " + position.getX());
            System.out.println("target Y => " + position.getY());

            return position;
            /*this.setHeadingAngle(0);

            m_XmotionPID.setSetpoint(position.getX());
            m_YmotionPID.setSetpoint(position.getY());
            //if(getAlliance() == Alliance.Blue) {
                this.drive( 
                    m_XmotionPID.calculate(m_RobotPose.getX()),
                    m_YmotionPID.calculate(m_RobotPose.getY()),
                    0, 
                    true );//m_Filter.calculate(m_RotationPID.calculate(m_Imu.getYaw(), position.getRotation().getDegrees())));
                    SmartDashboard.putNumber("XPID_output", m_XmotionPID.calculate(m_RobotPose.getX(), position.getX()));
                    SmartDashboard.putNumber("YPID_output", m_YmotionPID.calculate(m_RobotPose.getY(), position.getY()));
                    SmartDashboard.putNumber("check this", position.getX());*/
            //} else {
                /*this.drive(
                    -m_XmotionPID.calculate(m_RobotPose.getX(), position.getX()), 
                    -m_YmotionPID.calculate(m_RobotPose.getY(), position.getY()), 
                    0,
                    false);//m_Filter.calculate(m_RotationPID.calculate(m_Imu.getYaw(), position.getRotation().getDegrees())));
                    SmartDashboard.putNumber("output", m_XmotionPID.calculate(m_XmotionPID.calculate(m_RobotPose.getX(), position.getX())));
                    SmartDashboard.putNumber("check this", m_RobotPose.getX());      */     
           //}   
        }

        return m_RobotPose;
    } 

    public boolean atSetpoint() {
        return (m_XmotionPID.atSetpoint() && m_YmotionPID.atSetpoint() && m_RotationPID.atSetpoint());
    }

    /*ublic Pose2d getRightReefPose() {
        Pose3d targetPose = getTargetPose(min_REEFtagID);
        Pose2d targetpose2d;
        if(targetPose != null) {
            targetpose2d = targetPose.toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(targetpose2d.getRotation());
            Pose2d targetPose2d = targetPose.toPose2d();
            Translation2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetpose2d.getRotation().plus(Rotation2d.fromDegrees(90))));

            Pose2d position = new Pose2d(targetpose2d.getTranslation().plus(tagToPillar).plus(robotToEage), targetpose2d.getRotation().unaryMinus());
            return position;
        }
        return null;
    }

    public Pose2d getLeftReefPose() {
        Pose3d targetPose = getTargetPose(min_REEFtagID);
        if(targetPose != null) {
            Pose2d targetPose2d = getTargetPose(min_REEFtagID).toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(targetPose2d.getRotation());
            Translatiov              n2d tagToPillar = (new Translation2d(0.16, 0).rotateBy(targetPose2d.getRotation().minus(Rotation2d.fromDegrees(90))));

            Pose2d position = new Pose2d(targetPose2d.getTranslation().plus(tagToPillar).plus(robotToEage), targetPose2d.getRotation().unaryMinus());
            return position;
        }
        return null;
    }

    public Pose2d getCoralSPose() {
        Pose3d targetPose = getTargetPose(min_CoralStationtagID);
        if(targetPose != null){
            Pose2d tagpose = getTargetPose(min_CoralStationtagID).toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(tagpose.getRotation());

            Pose2d CSPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation());
            return CSPosition;
        }
        return null;
    }

    public Pose2d getBlueProcessorPose() {
        Pose3d targetPose = getTargetPose(3);
        if(targetPose != null){
            Pose2d tagpose = getTargetPose(3).toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(tagpose.getRotation());

            Pose2d BPPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation().unaryMinus());
            return BPPosition;
        }
        return null;
    }
    
    public Pose2d getRedProcessorPose() {
        Pose3d targetPose = getTargetPose(16);
        if(targetPose != null){
            Pose2d tagpose = getTargetPose(16).toPose2d();
            Translation2d robotToEage = (new Translation2d(SwerveConstants.khowlongismyrobot/2, 0.0)).rotateBy(tagpose.getRotation());

            Pose2d BPPosition = new Pose2d(tagpose.getTranslation().plus(robotToEage), tagpose.getRotation().unaryMinus());
            return BPPosition;
        }
        return null;
    }*/

    public Command switchDriveMode() {
        return runOnce(() -> {
            fieldOriented = !fieldOriented;
        });
    }

    public Command increaseSpeed() {
        return runOnce(() -> {
            if(this.maxSpeedRatio > 0.9) return;
            this.maxSpeedRatio+=0.1;
        });
    }

    public Command decreaseSpeed() {
        return runOnce(() -> {
            if(this.maxSpeedRatio < 0.2) return;
            this.maxSpeedRatio-=0.1;
        });
    }

    public Command tohighSpeed() {
        return runOnce(() -> {
            this.maxSpeedRatio = 0.8;
        });
    }

    public Command tolowspeed() {
        return runOnce(() -> {
            this.maxSpeedRatio = 0.5;
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
        if(Swerve.ffControl) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.kMaxVelocityMetersPerSecond*this.maxSpeedRatio);
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, this.maxSpeedRatio);
        }
        
        m_LeftFrontModule.setState(desiredState[0]);
        m_RightFrontModule.setState(desiredState[1]);
        m_LeftRearModule.setState(desiredState[2]);
        m_RightRearModule.setState(desiredState[3]);
    }

    public void updateVisionPose() {
        //double robotheading = m_alliance == Alliance.Blue ? m_AHRS.getRotation2d().getDegrees() + 180 : m_AHRS.getRotation2d().getDegrees();
        
        LimelightHelpers.SetRobotOrientation("limelight", this.getImuARotation2d().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        cameraGotSomething = LimelightHelpers.getTV("");

        if(cameraGotSomething && mt2 != null) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999999));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds); 
            m_poseEstimator.resetPosition(getImuARotation2d(), getModulePositions(), mt2.pose);
        }
        SmartDashboard.putBoolean("cameraGotSomething", cameraGotSomething);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(
            getImuARotation2d(),
            getModulePositions(),
            pose);
    }
    
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.kSwerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveChassis(ChassisSpeeds speeds) {
        driveChassis(
            -speeds.vxMetersPerSecond,
            -speeds.vyMetersPerSecond,
            -speeds.omegaRadiansPerSecond
        );
    }

    public void driveChassis(double xSpeed, double ySpeed, double zSpeed) {
        SwerveModuleState[] states = SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                xSpeed,
                ySpeed,
                zSpeed
            )
        );

        setModulestate(states);
    }

    public Pose3d getTargetPose(int ID) {
        var tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        if(tagPose.getTagPose(ID).isPresent()){
            return tagPose.getTagPose(ID).get();
        }
        return null;
    }

    public void resetPID() {
        m_XmotionPID.reset();
        m_YmotionPID.reset();
        m_RotationPID.reset();
    }

    public Alliance getAlliance() {
        var alliance = DriverStation.getAlliance();
        if(alliance != null ) {
            return alliance.get();
        }
        return null;
    }
}