package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;

public class VisionSubsystem extends SubsystemBase {
/*
    private NetworkTable m_table;
    
    private String m_tablename;
    private double m_pipeline = 0;

    public VisionSubsystem() {
        m_tablename = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tablename);
        var m_field = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();


    }

    public boolean getIsTargetVisible() {
        NetworkTableEntry tv = m_table.getEntry("tv");
        double v = tv.getDouble(0);
        if(v == 0.0f) {
            return false;
        } else {
            return true;
        }
    }

    public double getdegRotationToTarget() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        double x = tx.getDouble(0);
        return x;
    }

    public double getdegVerticalToTarget() {
        NetworkTableEntry ty = m_table.getEntry("ty");
        double y = ty.getDouble(0);
        return y;
    }

    public double getTargetArea() {
        NetworkTableEntry ta = m_table.getEntry("ta");
        double a = ta.getDouble(0);
        return a;
    }

    public double[] getCameraPose_TargetSpace() {
        NetworkTableEntry td = m_table.getEntry("targetpose_cameraspace");
        double[] distantce = td.getDoubleArray(new double[6]);
        return distantce;
    }

    public void increasePipeline() {
        m_pipeline++;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(m_pipeline);
    }

    public void decreasePipeline() {
        m_pipeline--;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(m_pipeline);
    }


    public double getTargetID() {
        NetworkTableEntry tid = m_table.getEntry("tid");
        double id = tid.getDouble(0);
        return id;
    }

    public double[] getBotPose() {
        NetworkTableEntry bp = m_table.getEntry("botpose_orb");
        double[] botpose = bp.getDoubleArray(new double[6]);
        return botpose;
    }

    public static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /**
     * Converts a Pose3d object to an array of doubles.
     * 
     * @param pose The Pose3d object to convert.
     * @return The array of doubles representing the pose.
     **/
    /*
    public static double[] pose3dToArray(Pose3d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = pose.getTranslation().getZ();
        result[3] = Units.radiansToDegrees(pose.getRotation().getX());
        result[4] = Units.radiansToDegrees(pose.getRotation().getY());
        result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
        return result;
    }
    
    /**
     * Converts a Pose2d object to an array of doubles.
     * 
     * @param pose The Pose2d object to convert.
     * @return The array of doubles representing the pose.
     **/
    /*
    public static double[] pose2dToArray(Pose2d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = 0;
        result[3] = Units.radiansToDegrees(0);
        result[4] = Units.radiansToDegrees(0);
        result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
        return result;
    }
    */
    //public

    //@Override
    //public void periodic() {
    
    
}
