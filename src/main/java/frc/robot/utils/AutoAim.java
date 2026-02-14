package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.GlobalVars;
import frc.robot.GlobalConstants.FIELD_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public class AutoAim {
    private static final double shotTime = 0.0;
    private final SwerveDriveState state;

    InterpolatingDoubleTreeMap hoodAngle = new InterpolatingDoubleTreeMap();

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

        double dx = tx - cx - (shotTime * state.Speeds.vxMetersPerSecond);
        double dy = ty - cy - (shotTime * state.Speeds.vyMetersPerSecond);

        double dist = fastSqrt(
            (float) (dx*dx + dy*dy)
        );

        // Logger.recordOutput("Visualization/AimTarget", new Translation2d(tx, ty));

        return new Pair<>(Math.atan2(dy, dx), dist);
    }
}
