package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.LimelightHelpers;

public class VisionLimelight implements VisionIO{
    private final String cameraName;
    private Pose2d pose2d;

    private final List<PoseObservation> m_results = new ArrayList<>();

    public VisionLimelight(String cameraName) {
        this.cameraName = cameraName;
    }
    
    @Override
    public void feedPose(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.SetRobotOrientation(cameraName, pose2d.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.cameraName);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.cameraName);
        
        m_results.clear();
        
        if (mt2 != null && (mt2.tagCount > 0)) {
            m_results.add(new PoseObservation(mt2.timestampSeconds, mt2.pose, LimelightHelpers.getMT2StdDevs(cameraName)));
        }

        if (mt1 != null && (mt1.tagCount > 0)) {
            m_results.add(new PoseObservation(mt1.timestampSeconds, mt1.pose, LimelightHelpers.getMT1StdDevs(cameraName)));
        }

        inputs.poseObservation = m_results;
    }
    
    @Override
    public String toString() {
        return this.cameraName;
    }
}
