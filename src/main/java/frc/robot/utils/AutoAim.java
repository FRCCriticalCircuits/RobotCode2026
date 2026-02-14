package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.drive.Drive;

public class AutoAim {
    private static final double shotTime = 0.0;
    private final SwerveDriveState state;

    InterpolatingDoubleTreeMap hoodAngle = new InterpolatingDoubleTreeMap();

    public AutoAim(Drive drive){
        this.state = drive.getState();
    }

    private double fastSqrt(float number) {
        int i = Float.floatToIntBits(number);
        i = 0x1fbd1df5 + (i >> 1);
        float y = Float.intBitsToFloat(i);
        return y;
    }

    public Pair<Double, Double> getAimParams(Pose2d targetPose){
        double dx = targetPose.getX() - state.Pose.getMeasureX().baseUnitMagnitude() + (shotTime * state.Speeds.vxMetersPerSecond);
        double dy = targetPose.getY() - state.Pose.getMeasureY().baseUnitMagnitude() + (shotTime * state.Speeds.vyMetersPerSecond);

        double dist = fastSqrt(
            (float) (dx*dx + dy*dy)
        );

        return new Pair<>(Math.atan2(dy, dx), dist);
    }
}
