package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LimelightHelpers;

public class VisionLimelight implements VisionIO{
    private final String cameraName;
    private Pose2d pose2d;
    private Matrix<N3, N1> stdDevs;

    public VisionLimelight(String cameraName, Matrix<N3, N1> stdDevs) {
        this.cameraName = cameraName;
        this.stdDevs = stdDevs;
    }

    @Override
    public Matrix<N3, N1> getStdDevs() {
        return this.stdDevs;
    }

    @Override
    public void feedPose(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.SetRobotOrientation(cameraName, pose2d.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.cameraName);
        
        if (mt2 != null) {
            if (mt2.tagCount == 0) inputs.poseObservation = null;
            inputs.poseObservation = new ArrayList<PoseObservation>() {{
                add(new PoseObservation(mt2.timestampSeconds, mt2.pose));
            }};
        } else {
            inputs.poseObservation = null;
        };
    }
    
    @Override
    public String toString() {
        return this.cameraName;
    }
}
