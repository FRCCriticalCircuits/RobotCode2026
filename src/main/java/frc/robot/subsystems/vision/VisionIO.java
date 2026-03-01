package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    public static class VisionIOInputs {
        public boolean connected = false;
        public List<PoseObservation> poseObservation = null;
    }

    public static record PoseObservation(
        double timeStamp,
        Pose2d robotPose
    ) {}

    public default void updateInputs(VisionIOInputs inputs) {}
}