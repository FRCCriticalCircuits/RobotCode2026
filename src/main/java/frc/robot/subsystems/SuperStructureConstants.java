package frc.robot.subsystems;

/**
 * All units should be in radians
 * Kraken Implementations will convert it to rotations,
 * which CTRE requires for their API
 */
public class SuperStructureConstants {
    // public static final double INTAKE_ARM_UP = (0.25 - 0.081) * Math.PI * 2;
    // public static final double INTAKE_ARM_DOWN = -0.0138 * Math.PI * 2;
    public static final double INTAKE_ARM_DOWN = -0.239; // in rot

    // TODO tune-superstructure: verify intake roller speed(radians) under load.
    public static final double INTAKE_ROLLER_VEL = 40 * 2 * Math.PI; // 45 rotation maximum

    // Hopper Velocity, tuned with Feedforward so it's easier to adjust speed to feed the ball into the shooter
    public static final double HOPPER_VELOCITY = 55.0 * Math.PI * 2;

    public static final double CLIMBER_FORWARD = 8.0;
    public static final double CLIMBER_REVERSE = -8.0;
}
