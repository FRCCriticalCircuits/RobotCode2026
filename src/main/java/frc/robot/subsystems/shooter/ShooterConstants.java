package frc.robot.subsystems.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterConstants {
    public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
    // Estimated hood rotational inertia for simulation modeling.
    public static final double HOOD_MOI = SingleJointedArmSim.estimateMOI(
        0.2032,
        0.90718474
    );
    public static final LinearSystem<N2, N1, N2> HOOD_STATE_SPACE_TEMP = LinearSystemId.createDCMotorSystem(
        ShooterConstants.HOOD_GEARBOX,
        ShooterConstants.HOOD_MOI,
        HAL.HOOD_GEARING
    );
    public static final LinearSystem<N2, N1, N2> HOOD_STATE_SPACE = HOOD_STATE_SPACE_TEMP;

    public static final DCMotor SHOOTER_GEARBOX = DCMotor.getKrakenX60Foc(2);
    // Approximate shooter wheel+axle properties for state-space simulation.
    public static final double WHEEL_AXLE_MASS = Units.lbsToKilograms(8);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
    public static final double SHOOTER_MOI = (1.0 / 2.0) * WHEEL_AXLE_MASS * Math.pow(WHEEL_RADIUS, 2);
    public static final LinearSystem<N2, N1, N2> SHOOTER_STATE_SPACE_TEMP = LinearSystemId.createDCMotorSystem(
        ShooterConstants.SHOOTER_GEARBOX,
        ShooterConstants.SHOOTER_MOI,
        HAL.SHOOTER_GEARING
    );
    public static final LinearSystem<N2, N1, N2> SHOOTER_STATE_SPACE = SHOOTER_STATE_SPACE_TEMP;

    public class HAL{
        public static final double HOOD_GEARING = 40;
        public static final double SHOOTER_GEARING = 24.0 / 18.0;

        public static final boolean HOOD_INVERT = true;
        public static final boolean SHOOTER_INVERT = false;
        public static final boolean SECONDARY_SHOOTER_INVERT = false;

        public static final double HOOD_PID_P = 0;
        public static final double HOOD_PID_I = 0;
        public static final double HOOD_PID_D = 0;

        public static final double SHOOTER_PID_P = 0;
        public static final double SHOOTER_PID_I = 0;
        public static final double SHOOTER_PID_D = 0;
    }

    public class Tuning{
        // Allowed control error before shooter is considered ready to feed.
        public static final double HOOD_STABLE_TOLERANCE_RAD = Math.toRadians(1.5);
        public static final double SHOOTER_STABLE_TOLERANCE_RAD_PER_SEC = 8.0;

        // Hood torque current caps used in HOOD Kraken configuration.
        public static final double HOOD_PEAK_FORWARD_TORQUE_CURRENT = 30.0;
        public static final double HOOD_PEAK_REVERSE_TORQUE_CURRENT = -30.0;

        // Hood torque current caps used in Shooter Kraken configuration.
        public static final double SHOOTER_PEAK_FORWARD_TORQUE_CURRENT = 40.0;
        public static final double SHOOTER_PEAK_REVERSE_TORQUE_CURRENT = -40.0;
    }

    public class CAD{
        public static final double ORIGIN_OFFSET_X = -0.146;
        public static final double ORIGIN_OFFSET_Y = 0;
        public static final double ORIGIN_OFFSET_Z = 0.445;

        public static final double PITCH_OFFSET = Math.toRadians(-24.34);
    }
}
