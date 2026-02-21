package frc.robot.subsystems.drive;

import java.nio.BufferOverflowException;
import java.nio.BufferUnderflowException;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class SwerveTelemetry {
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public SwerveTelemetry() {}
    
    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        try {
            Logger.recordOutput("Swerve/OdometryFreq", 1.0 / state.OdometryPeriod);
            Logger.recordOutput("Swerve/OdometryPose", state.Pose);
            Logger.recordOutput("Swerve/CurrentModuleStates", state.ModuleStates);
            Logger.recordOutput("Swerve/TargetModuleStates", state.ModuleTargets);
        } catch (BufferOverflowException e) {
            System.out.print("Buffer Overflow @ SwerveTelemetry.java Logging");
        } catch (BufferUnderflowException e) {
            System.out.print("Buffer Underflow @ SwerveTelemetry.java Logging");
        }
    }
}
