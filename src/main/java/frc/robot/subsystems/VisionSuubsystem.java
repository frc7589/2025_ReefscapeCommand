package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSuubsystem extends SubsystemBase {
    // TODO
    private NetworkTable m_table;
    private String m_tablename;
    private double m_pipeline = 0;

    public VisionSuubsystem() {
        m_tablename = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tablename);
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

    public double getCameraPose_TargetSpace(int pose) {
        NetworkTableEntry td = m_table.getEntry("targetpose_cameraspace");
        double[] distantce = td.getDoubleArray(new double[6]);
        return distantce[pose];
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
}
