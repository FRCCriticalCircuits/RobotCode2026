package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.GlobalConstants;
import frc.robot.GlobalVars;
import frc.robot.GlobalConstants.FIELD_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public class AutoAim {
    private static final double shotTime = 0.1;
    private final SwerveDriveState state;

    InterpolatingDoubleTreeMap hoodAngle = new InterpolatingDoubleTreeMap();

    private final Transform2d heading = new Transform2d(new Translation2d(5, 0), Rotation2d.kZero);

    public AutoAim(Drive drive){
        this.state = drive.getState();
    }

    private double fastSqrt(float number) {
        int i = Float.floatToIntBits(number);
        i = 0x1fbd1df5 + (i >> 1);
        float y = Float.intBitsToFloat(i);
        return y;
    }

    public Pair<Double, Double> getAimParams(){
        double tx, ty, cx, cy;

        // Current Positions
        cx = state.Pose.getMeasureX().baseUnitMagnitude();
        cy = state.Pose.getMeasureY().baseUnitMagnitude();

        if(GlobalVars.BLUE_ALLIANCE){
            if(cx < FIELD_CONSTANTS.BLUE_ALLIANCE_ZONE_X){
                tx = FIELD_CONSTANTS.BLUE_HUB.getX();
                ty = FIELD_CONSTANTS.BLUE_HUB.getY();
            }else if(cy > (FIELD_CONSTANTS.FIELD_WIDTH / 2)){
                tx = FIELD_CONSTANTS.TOP_LEFT.getX();
                ty = FIELD_CONSTANTS.TOP_LEFT.getY();
            }else{
                tx = FIELD_CONSTANTS.BOTTOM_LEFT.getX();
                ty = FIELD_CONSTANTS.BOTTOM_LEFT.getY();
            }
        }else{
            if(cx > FIELD_CONSTANTS.RED_ALLIANCE_ZONE_X){
                tx = FIELD_CONSTANTS.RED_HUB.getX();
                ty = FIELD_CONSTANTS.RED_HUB.getY();
            }else if(cy > (FIELD_CONSTANTS.FIELD_WIDTH / 2)){
                tx = FIELD_CONSTANTS.TOP_RIGHT.getX();
                ty = FIELD_CONSTANTS.TOP_RIGHT.getY();
            }else{
                tx = FIELD_CONSTANTS.BOTTOM_RIGHT.getX();
                ty = FIELD_CONSTANTS.BOTTOM_RIGHT.getY();
            }
        }

        // Current ChassisSpeed
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        // rotation Field-centric: state.Speeds.omegaRadiansPerSecond
        
        double dx = tx - cx - (shotTime * fieldSpeeds.vxMetersPerSecond);
        double dy = ty - cy - (shotTime * fieldSpeeds.vyMetersPerSecond);

        double dist = fastSqrt(
            (float) (dx*dx + dy*dy)
        );

        /* Visualization */
        if(!GlobalConstants.COMP){
            Pose2d futurPose2d = new Pose2d(-dx + tx, -dy + ty, Rotation2d.fromRadians(Math.atan2(dy, dx)));

            Logger.recordOutput("Visualization/AimTarget", new Translation2d(tx, ty));
            Logger.recordOutput("Visualization/FuturePose", futurPose2d);
            Logger.recordOutput("Visualization/FutureHeading", futurPose2d.plus(heading));
            Logger.recordOutput("Visualization/Speeds", fieldSpeeds);
        }
        
        return new Pair<>(Math.atan2(dy, dx), dist);
    }
}
