package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded
    );

    // TODO tune-photonvision: adjust by camera quality and tag count.
    public static final Matrix<N3, N1> PV_STDDEV_BASE = VecBuilder.fill(0.7, 0.7, 1.5);
    public static final double PV_AMBIGUITY_LIMIT = 0.15;
    // baseStdDev * (1 + PV_SCALING * dist^2)
    public static final double PV_SCALING = 0.5; 

    // works a little bit different
    // estimated stdDev from LL3G * SCLAING_CONSTANT
    public static final double LL_MT1_SCALING = 2.0; 
    public static final double LL_MT2_SCALING = 1.5;

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
