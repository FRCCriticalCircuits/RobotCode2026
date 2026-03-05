package frc.robot.utils.calc;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        // check if the return value is within the cone
        // O: origin of WPILIB coordinate system
        // P: the apex of the cone
        // n: the base vector with its origin at P,
        //      opposite direction to heading given by Pose2d, with a length of 1 
        // Q: robot's current position
        // PQ: the vector PQ
        // a: the limit, open angle of the cone

        Translation2d pq = state.Pose.getTranslation().minus(ret.getTranslation());
        Translation2d n = new Translation2d(1, 0).rotateBy(ret.getRotation().plus(Rotation2d.k180deg));
        double dotProduct = pq.dot(n) / pq.getSquaredNorm();

        // if the limit is not satisfied, fall back to current position
        if(dotProduct < CalculatorConstants.COS_A_LIMIT){
            return state.Pose;
        }

        return ret;
    }
}
