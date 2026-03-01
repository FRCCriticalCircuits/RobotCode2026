package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded
    );

    public static final Transform3d leftCam = new Transform3d(
        -0.258,
        0.264,
        0.185,
        new Rotation3d(
            Math.toRadians(0),
            Math.toRadians(-30),
            Math.toRadians(135)
        )
    );

    public static final Transform3d rightCam = new Transform3d(
        -0.258,
        -0.264,
        0.185,
        new Rotation3d(
            Math.toRadians(0),
            Math.toRadians(-30),
            Math.toRadians(-135)
        )
    );
}
