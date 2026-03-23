package frc.robot.utils.calc;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterConstants.TUNING;

public class CalculatorConstants {
    // TODO tune for actual robot
    public static final double shotTime = 0.3;
    public static final double difference = 0.005;

    // TODO tune-LUT
    // max 0.11, min 0.005 for rotations
    // these values should be in radians
    public static InterpolatingDoubleTreeMap hoodAngle = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, (TUNING.HOOD_MIN_POSITION_ROT + difference) * Math.PI * 2.0),
        Map.entry(3.0, (TUNING.HOOD_MIN_POSITION_ROT + 6 * difference) * Math.PI * 2.0),
        Map.entry(6.0, (TUNING.HOOD_MAX_POSITION_ROT) * Math.PI * 2.0) 
    );

    public static InterpolatingDoubleTreeMap shooterVelocity = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, AimCalc.calculateRequiredVelocity(0.0, (TUNING.HOOD_MIN_POSITION_ROT + difference) * Math.PI * 2.0)),
        Map.entry(3.0, AimCalc.calculateRequiredVelocity(3.0, (TUNING.HOOD_MIN_POSITION_ROT + 6 * difference) * Math.PI * 2.0)),
        Map.entry(6.0, AimCalc.calculateRequiredVelocity(5.5, (TUNING.HOOD_MAX_POSITION_ROT) * Math.PI * 2.0)) 
    );

    // TODO decide limit (in degress) of the cone-shape check for climbing calculator
    public static double COS_A_LIMIT = Math.cos(Math.toRadians(30)); 
}
