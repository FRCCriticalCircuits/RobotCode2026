package frc.robot.utils.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.vision.VisionConstants;

public class AdvancedPhotonEstimator extends PhotonPoseEstimator{
    private AprilTagFieldLayout fieldTags;
    private Transform3d robotToCamera;

    public AdvancedPhotonEstimator(AprilTagFieldLayout fieldTags, Transform3d robotToCamera){
        super(fieldTags, robotToCamera);

        this.fieldTags = fieldTags;
        this.robotToCamera = robotToCamera;
    }

    private static double sumArray(double[] arr) {
        double sum = 0;
        for (double v : arr) sum += v;
        return sum;
    }

    /**
     * @param cameraResult A pipeline result from the camera.
     * @return Whether or not pose estimation should be performed.
     */
    private boolean shouldEstimate(PhotonPipelineResult cameraResult) {
        // Time in the past -- give up, since the following if expects times > 0
        if (cameraResult.getTimestampSeconds() < 0) {
            return false;
        }

        // If no targets seen, trivial case -- can't do estimation
        return cameraResult.hasTargets();
    }

    /**
     * ChatGPT wrote this javadoc comment and inverse-variance weighting stuff
     * Estimate the standard deviation (uncertainty) of a vision-based pose measurement
     * using a list of {@link PhotonTrackedTarget} objects.
     *
     * The stdDev in X, Y, and yaw directions is computed using a distance-based heuristic algorithm:
     *  - stdDev grows with distance squared.
     *  - Multiple tags are combined using inverse-variance weighting
     *  - The baseStdDevs represent minimal expected measurement noise when the tag is near and front-facing.
     *  - yaw uncertainty can be increased for side-view tags to account for less reliable PnP rotation estimates.
     * 
     * @param targetsUsed List of PhotonTrackedTarget objects used for pose estimation; should be pre-filtered for ambiguity.
     * @return 3x1 Matrix<N3,N1> representing the estimated standard deviations [sigmaX, sigmaY, sigmaTheta].
     */
    public static Matrix<N3,N1> estimateStdDev(List<PhotonTrackedTarget> targetsUsed, Matrix<N3, N1> baseStdDevs, double scaling){
        double[] sigmaXInv2 = new double[targetsUsed.size()];
        double[] sigmaYInv2 = new double[targetsUsed.size()];
        double[] sigmaThetaInv2 = new double[targetsUsed.size()];

        for (int i = 0; i < targetsUsed.size(); i++) {
            PhotonTrackedTarget target = targetsUsed.get(i);

            Translation3d targetTranslation = target.bestCameraToTarget.getTranslation();
            double dist_square = targetTranslation.getSquaredDistance(Translation3d.kZero);
            
            // apply a scale (scale * dist^2) over the base stdDev
            double sigmaX = baseStdDevs.get(0, 0) * (1 + scaling * dist_square);
            double sigmaY = baseStdDevs.get(1, 0) * (1 + scaling * dist_square);
            double sigmaTheta = baseStdDevs.get(2, 0) * (1 + scaling * dist_square);

            // Increase yaw stdDev when the tag is viewed from a large side angle,
            // because PnP rotation estimate is less reliable for side-view tags.
            sigmaTheta *= (1 + Math.abs(target.yaw) / 90.0);

            sigmaXInv2[i] = 1.0 / (sigmaX * sigmaX);
            sigmaYInv2[i] = 1.0 / (sigmaY * sigmaY);
            sigmaThetaInv2[i] = 1.0 / (sigmaTheta * sigmaTheta);
        }

        double sigmaXCombined = 1.0 / Math.sqrt(sumArray(sigmaXInv2));
        double sigmaYCombined = 1.0 / Math.sqrt(sumArray(sigmaYInv2));
        double sigmaThetaCombined = 1.0 / Math.sqrt(sumArray(sigmaThetaInv2));

        return VecBuilder.fill(sigmaXCombined, sigmaYCombined, sigmaThetaCombined);
    }

    /**
     * <h2> 
     * Modified Version! it throw the result when ambiguity difference is too large
     * </h2>
     */
    @Override
    public Optional<EstimatedRobotPose> estimateCoprocMultiTagPose(PhotonPipelineResult cameraResult) {
        if (cameraResult.getMultiTagResult().isEmpty() || !shouldEstimate(cameraResult)) {
            return Optional.empty();
        }

        // ambiguity is the reprojection err of the `best` (the one with lower repj err) result
        // divide by `alt` (the one with higher repj err). 
        // Reporjection Error ranges from 0-1(normally, not sure if PhotonVision does this)
        // Therefore, drop result >= Ambiguity Limit because higher ambiguity means that the difference 
        // between to result are too similar.
        if(cameraResult.getMultiTagResult().get().estimatedPose.ambiguity >= VisionConstants.PV_AMBIGUITY_LIMIT){
            return Optional.empty();
        }

        // the result called `best` refers to reprojection error
        // maybe add some verification for both results if got time?
        var best_tf = cameraResult.getMultiTagResult().get().estimatedPose.best;
        var best = Pose3d.kZero
            .plus(best_tf) // field-to-camera
            .relativeTo(fieldTags.getOrigin())
            .plus(robotToCamera.inverse()); // field-to-robot
                        
        return Optional.of(
            new EstimatedRobotPose(
                best,
                cameraResult.getTimestampSeconds(),
                cameraResult.getTargets(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
    }

    /**
     * Return the estimated position of the robot with the lowest position ambiguity from a pipeline
     * result.
     * 
     * <h2> 
     * Modified Version! it only returns the final chosen pose in the list of tag used
     * </h2>
     *
     * @param cameraResult A pipeline result from the camera.
     * @return An {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
     *     create the estimate, or an empty optional if there's no targets.
     */
    @Override
    public Optional<EstimatedRobotPose> estimateLowestAmbiguityPose(
            PhotonPipelineResult cameraResult) {
        if (!shouldEstimate(cameraResult)) {
            return Optional.empty();
        }
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : cameraResult.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
            // reportFiducialPoseError(targetFiducialId);
            return Optional.empty();
        }

        return Optional.of(
            new EstimatedRobotPose(
                targetPosition
                        .get()
                        .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse()),
                cameraResult.getTimestampSeconds(),
                List.of(lowestAmbiguityTarget), // cameraResult.getTargets(),
                PoseStrategy.LOWEST_AMBIGUITY));
    }
}
