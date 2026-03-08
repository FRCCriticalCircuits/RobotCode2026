package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;

public interface VisionIO {
    public static class VisionIOInputs {
        public boolean connected = false;
        public List<PoseObservation> poseObservation = null;
    }

    public static record PoseObservation(
        double timeStamp,
        Pose2d robotPose,
        Matrix<N3, N1> stdDev
    ) {}

    public default void feedPose(Pose2d pose2d) {}
    public default void updateInputs(VisionIOInputs inputs) {}
}