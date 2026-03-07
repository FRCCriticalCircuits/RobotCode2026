package frc.robot.subsystems;

public class SuperStructureConstants {
    public static final double INTAKE_ARM_UP = 10;
    public static final double INTAKE_ARM_DOWN = 45.7;

    // TODO tune-superstructure: verify intake roller speed under load.
    public static final double INTAKE_ROLLER_VEL = 300;

    // TODO tune-superstructure: this was estimated from ~8V operation.
    public static final double SHOOT_FLYWHEEL_VEL = 360.0;
    // Hopper voltage is intentionally left as-is.
    public static final double SHOOT_SEQUENCER_VOLTS = 2.0;

    // TODO tune-superstructure: verify climb voltages with final mechanism load.
    public static final double CLIMBER_FORWARD = 8.0;
    public static final double CLIMBER_REVERSE = -8.0;
}
