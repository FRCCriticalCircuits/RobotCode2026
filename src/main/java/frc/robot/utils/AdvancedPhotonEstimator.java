package frc.robot.utils;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class AdvancedPhotonEstimator extends PhotonPoseEstimator{
    public AdvancedPhotonEstimator(AprilTagFieldLayout fieldTags, Transform3d robotToCamera){
        super(fieldTags, robotToCamera);
    }

    
}
