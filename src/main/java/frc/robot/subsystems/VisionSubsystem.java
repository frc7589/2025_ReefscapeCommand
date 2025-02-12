package frc.robot.subsystems;


import java.util.List;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera m_Camera;

    private PhotonPipelineResult cameraResult;

    public VisionSubsystem() {
        this.m_Camera = new PhotonCamera("");
    }

    @Override
    public void periodic() {
        if(m_Camera != null) {
            this.cameraResult = m_Camera.getLatestResult();
        }
    }
    public boolean hasTargets() {
        return cameraResult.hasTargets();
    }

    public static boolean hasAPTag() {
        return LimelightHelpers.getTV("");
    }

    public Translation3d getTargetPose() {
        return cameraResult.getBestTarget().getBestCameraToTarget().getTranslation();
    }
}
