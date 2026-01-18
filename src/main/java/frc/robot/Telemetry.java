package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class Telemetry {
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry() {}
    
    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        // redo later
        // field Pose(Pose Estimator) state.Pose
        // Module States (target/current) state.ModuleTargets/ModuleStates
        // Odometry MetaData (Period) 1.0 / state.OdometryPerio
    }
}
