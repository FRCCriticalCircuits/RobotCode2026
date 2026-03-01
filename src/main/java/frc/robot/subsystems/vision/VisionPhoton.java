package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionPhoton implements VisionIO {
    private final String cameraName;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final List<PhotonPipelineResult> rawResults = new ArrayList<>();
    private final List<PoseObservation> result = new ArrayList<>();

    public VisionPhoton(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout, robotToCamera);

        this.cameraName = cameraName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        rawResults.clear();
        result.clear();
        rawResults.addAll(camera.getAllUnreadResults());
        
        for(PhotonPipelineResult rawResult: rawResults){
            photonEstimator.estimateCoprocMultiTagPose(rawResult)
                .or(() -> photonEstimator.estimateLowestAmbiguityPose(rawResult))
                .ifPresent(est -> result.add(
                    new PoseObservation(
                        est.timestampSeconds,
                        est.estimatedPose.toPose2d()
                    )
                ));
        }

        inputs.poseObservation = new ArrayList<>(result);
        // TODO implement isConnected
    }

    @Override
    public String toString() {
        return this.cameraName;
    }
}