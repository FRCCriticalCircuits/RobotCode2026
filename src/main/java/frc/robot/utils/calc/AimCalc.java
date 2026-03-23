package frc.robot.utils.calc;

import java.nio.BufferOverflowException;
import java.nio.BufferUnderflowException;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.GlobalConstants;
import frc.robot.GlobalVars;
import frc.robot.GlobalConstants.FIELD_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public class AimCalc {
    private final SwerveDriveState state;

    private final Transform2d heading = new Transform2d(new Translation2d(5, 0), Rotation2d.kZero);

    public AimCalc(Drive drive){
        this.state = drive.getState();
    }

    private static double fastSqrt(float number) {
        int i = Float.floatToIntBits(number);
        i = 0x1fbd1df5 + (i >> 1);
        float y = Float.intBitsToFloat(i);
        return y;
    }

    public static double metersPerSecondToRPM(double velocityMetersPerSec) {
        double wheelDiameterMeters = 0.1016; // TODO
        double slipFactor = 0.5; // Typical for single-wheel + hood shooters

        // 1. Calculate the required surface velocity of the wheel
        double wheelSurfaceVelocity = velocityMetersPerSec / slipFactor;

        // 2. Convert surface velocity to RPM
        // RPM = (Velocity / Circumference) * 60
        double rpm = (wheelSurfaceVelocity / (Math.PI * wheelDiameterMeters)) * 60;

        return rpm;
    }

    public static double calculateRequiredVelocity(double distanceMeters, double angleRadians) {
        double g = 9.81;
        double x = distanceMeters;
        double theta = angleRadians;
        double y = FIELD_CONSTANTS.HUB_HEIGHT - FIELD_CONSTANTS.SHOOTER_HEIGHT; // The vertical gap

        // The denominator of the trajectory fraction
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y);

        if (denominator <= 0) {
            // This means the angle is too low to ever reach the height 'y' at that distance
            return 0.0;
        }

        double vSquared = (g * Math.pow(x, 2)) / denominator;
        return metersPerSecondToRPM(fastSqrt((float) vSquared));
    }

    public class ShootingParams{
        public double yaw = 0;
        public double yaw_ff = 0;
        public double pitchRads = 0;
        public double velocityRadsPerSec = 0;
    }

    public ShootingParams getAimParams(){
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
        
        double dx = tx - cx - (CalculatorConstants.shotTime * fieldSpeeds.vxMetersPerSecond);
        double dy = ty - cy - (CalculatorConstants.shotTime * fieldSpeeds.vyMetersPerSecond);

        // Derivative of atan2
        double r_square = dx*dx + dy*dy;
        double rotationFF = 0;
        if (r_square > 1e-9) {
            rotationFF = (dx * fieldSpeeds.vyMetersPerSecond - dy * fieldSpeeds.vxMetersPerSecond) / r_square; 
            rotationFF *= -1;
        }

        double dist = fastSqrt(
            (float) (dx*dx + dy*dy)
        );

        /* Visualization */
        if(!GlobalConstants.COMP){
            Pose2d futurPose2d = new Pose2d(-dx + tx, -dy + ty, Rotation2d.fromRadians(Math.atan2(dy, dx)));

            try {
                Logger.recordOutput("Calc/AutoAimDist", dist);
                Logger.recordOutput("Visualization/AimTarget", new Pose2d(tx, ty, Rotation2d.kZero));
                Logger.recordOutput("Visualization/CurrentHeading", state.Pose.plus(heading));
                Logger.recordOutput("Visualization/FuturePose", futurPose2d);
                Logger.recordOutput("Visualization/FutureHeading", futurPose2d.plus(heading));
            } catch (BufferOverflowException e) {
                System.out.print("Buffer Overflow @ AutoAim.java Logging");
            } catch (BufferUnderflowException e){
                System.out.print("Buffer UnderFlow @ AutoAim.java Logging");
            }
        }

        final var ret = new ShootingParams();
        ret.yaw = Math.atan2(dy, dx);
        ret.yaw_ff = rotationFF;
        ret.pitchRads = CalculatorConstants.hoodAngle.get(dist);
        ret.velocityRadsPerSec = CalculatorConstants.shooterVelocity.get(dist);
        
        return ret;
    }
}
