package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import frc.robot.subsystems.drive.Drive;

public class AimCalculation {
    private final Drive drive;
    private static final double shotTime = 0.0;

    public AimCalculation(Drive drive){
        this.drive = drive;
    }

    public Pair<Double, Double> getAimParams(){
        SwerveDriveState state = drive.getState();
        
        // double newX = state.Pose.getMeasureX().baseUnitMagnitude() + (shotTime * state.Speeds.vxMetersPerSecond);
        // double newY = state.Pose.getMeasureY().baseUnitMagnitude() + (shotTime * state.Speeds.vyMetersPerSecond);



        double yaw = 0.0;
        double pitch = 0.0;

        return new Pair<>(yaw, pitch);
    }
}
