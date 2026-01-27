package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterConstants {    
    public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
    public static final double HOOD_MOI = SingleJointedArmSim.estimateMOI(
        0.2032,
        0.90718474
    );

    public static final DCMotor SHOOTER_GEARBOX = DCMotor.getKrakenX60Foc(2);    
    public static final double WHEEL_AXLE_MASS = Units.lbsToKilograms(8);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
    public static final double SHOOTER_MOI = (1.0 / 2.0) * WHEEL_AXLE_MASS
        * Math.pow(WHEEL_RADIUS, 2);

    public class HAL{
        public static final double HOOD_GEARING = 40;
        public static final double SHOOTER_GEARING = 24.0 / 18.0;
    }

    public class CAD{
        public static final double ORIGIN_OFFSET_X = -0.146;
        public static final double ORIGIN_OFFSET_Y = 0; 
        public static final double ORIGIN_OFFSET_Z = 0.445;

        public static final double PITCH_OFFSET = Math.toRadians(-24.34);
    }
}
