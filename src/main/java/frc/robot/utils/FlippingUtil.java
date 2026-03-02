/*
 * Vast majority of this file was copied from pathplanner:
 * pathplannerlib/src/main/java/com/pathplanner/lib/util/FlippingUtil.java
 */

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants.FIELD_CONSTANTS;

/** Utility class for flipping positions/rotations to the other side of the field */
public class FlippingUtil {
    /** Enum representing the different types of field symmetry */
    public enum FieldSymmetry {
        /**
         * Field is rotationally symmetric. i.e. the red alliance side is the blue alliance side rotated
         * by 180 degrees
         */
        kRotational,
        /** Field is mirrored vertically over the center of the field */
        kMirrored
    }

    /**
     * Flip a field position to the other side of the field, maintaining a blue alliance origin
     *
     * @param pos The position to flip
     * @return The flipped position
     */
    public static Translation2d flipFieldPosition(Translation2d pos) {
        return switch (FIELD_CONSTANTS.symmetryType) {
            case kMirrored -> new Translation2d(FIELD_CONSTANTS.FIELD_LENGTH - pos.getX(), pos.getY());
            case kRotational -> new Translation2d(FIELD_CONSTANTS.FIELD_LENGTH - pos.getX(), FIELD_CONSTANTS.FIELD_WIDTH - pos.getY());
        };
    }

    /**
     * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
     *
     * @param rotation The rotation to flip
     * @return The flipped rotation
     */
    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return switch (FIELD_CONSTANTS.symmetryType) {
            case kMirrored -> Rotation2d.kPi.minus(rotation);
            case kRotational -> rotation.minus(Rotation2d.kPi);
        };
    }

    /**
     * Flip a field pose to the other side of the field, maintaining a blue alliance origin
     *
     * @param pose The pose to flip
     * @return The flipped pose
     */
    public static Pose2d flipFieldPose(Pose2d pose) {
        return new Pose2d(flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
    }
}