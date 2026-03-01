package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
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