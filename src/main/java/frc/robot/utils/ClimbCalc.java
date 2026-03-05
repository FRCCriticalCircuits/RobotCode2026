package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.GlobalConstants.FIELD_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public class ClimbCalc {
    private final SwerveDriveState state;

    public ClimbCalc(Drive drive){
        this.state = drive.getState();
    }

    public Pose2d nearestClimbPos(){
        double minDist = Double.MAX_VALUE;
        Pose2d ret = state.Pose;
        for(Pose2d pose: FIELD_CONSTANTS.CLIMB_POSITIONS){
            double dist = pose.getTranslation().getDistance(state.Pose.getTranslation());
            if(dist < minDist){
                ret = pose;
                minDist = dist;
            }
        }
        return ret;
    }
}
