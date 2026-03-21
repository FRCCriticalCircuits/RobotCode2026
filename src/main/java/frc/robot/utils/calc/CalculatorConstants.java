package frc.robot.utils.calc;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterConstants.TUNING;

public class CalculatorConstants {
    // TODO tune for actual robot
    public static final double shotTime = 0.3;

    // TODO tune-LUT
    // max 0.11, min 0.005 for rotations
    // these values should be in radians
    public static InterpolatingDoubleTreeMap hoodAngle = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, TUNING.HOOD_MIN_POSITION_RAD),
        Map.entry(3.0, TUNING.HOOD_MIN_POSITION_RAD),
        Map.entry(8.0, TUNING.HOOD_MAX_POSITION_RAD) // passing
    );

    public static InterpolatingDoubleTreeMap shooterVelocity = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.7, 42.3 * Math.PI * 2),    
        Map.entry(1.953, 43.3 * Math.PI * 2),
        Map.entry(3.04, 49.5 * Math.PI * 2)
    );

    // TODO decide limit (in degress) of the cone-shape check for climbing calculator
    public static double COS_A_LIMIT = Math.cos(Math.toRadians(30)); 
}
