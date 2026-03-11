package frc.robot.subsystems;

/**
 * All units should be in radians
 * Kraken Implementations will convert it to rotations,
 * which CTRE requires for their API
 */
public class SuperStructureConstants {
    public static final double INTAKE_ARM_UP = 0.1;
    public static final double INTAKE_ARM_DOWN = 0.1;

    // TODO tune-superstructure: verify intake roller speed under load.
    public static final double INTAKE_ROLLER_VEL = 40; // 45 maximum

    // TODO tune-superstructure: this was estimated from ~8V operation.
    public static final double SHOOT_FLYWHEEL_VEL = 57.0; // Maximum
    // Hopper Velocity, Tuned Feedforward so it's easier to adjust speed to feed the ball into the shooter
    public static final double HOPPER_VELOCITY = 0.0;

    // TODO tune-superstructure: verify climb voltages with final mechanism load.
    public static final double CLIMBER_FORWARD = 8.0;
    public static final double CLIMBER_REVERSE = -8.0;
}
