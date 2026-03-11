package frc.robot.subsystems;

/**
 * All units should be in radians
 * Kraken Implementations will convert it to rotations,
 * which CTRE requires for their API
 */
public class SuperStructureConstants {
    public static final double INTAKE_ARM_UP = (-0.081) * Math.PI * 2;
    public static final double INTAKE_ARM_DOWN = (-0.25) * Math.PI * 2;

    // TODO tune-superstructure: verify intake roller speed(radians) under load.
    public static final double INTAKE_ROLLER_VEL = 40 * Math.PI * 2; // 45 rotation maximum

    // TODO tune-superstructure: this was estimated from ~8V operation.
    public static final double SHOOT_FLYWHEEL_VEL = 55.0 * Math.PI * 2; // Maximum
    // Hopper Velocity, Tuned Feedforward so it's easier to adjust speed to feed the ball into the shooter
    public static final double HOPPER_VELOCITY = 55.0 * Math.PI * 2;

    // TODO tune-superstructure: verify climb voltages with final mechanism load.
    public static final double CLIMBER_FORWARD = 8.0;
    public static final double CLIMBER_REVERSE = -8.0;
}
