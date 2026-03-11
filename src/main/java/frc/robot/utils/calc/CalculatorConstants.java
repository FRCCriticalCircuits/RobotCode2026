package frc.robot.utils.calc;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class CalculatorConstants {
    // TODO tune for actual robot
    public static final double shotTime = 0.1;

    // TODO tune-aim: replace with measured distance-to-hood-angle interpolation data.
    // max 0.11, min 0.005 for rotations
    // these values should be in radians
    public static InterpolatingDoubleTreeMap hoodAngle = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 0.01 * Math.PI * 2),
        Map.entry(3.0, 0.10 * Math.PI * 2),
        Map.entry(30.0, 0.10 * Math.PI * 2)
    );

    // TODO decide open angle of the cone check for climbing position calculator
    public static double COS_A_LIMIT = Math.cos(Math.toRadians(30)); 
}
