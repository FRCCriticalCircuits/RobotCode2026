package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ChassisConstants {
    //#region modifiable
    private class SIMULATION {
        // TODO tune-drive: PathPlanner/AutoDrive translation PID for simulation.
        private static final double TRANSLATION_PID_P = 5.0;
        private static final double TRANSLATION_PID_D = 0.0;
        // TODO tune-drive: PathPlanner/AutoDrive rotation PID for simulation.
        private static final double ROTATION_PID_P = 15.0;
        private static final double ROTATION_PID_D = 0.0;
        // TODO tune-drive: AutoDrive trapezoid limits (m/s and m/s^2).
        private static final double AUTO_DRIVE_MAX_VEL = 0.5;
        private static final double AUTO_DRIVE_MAX_ACCEL = 1.0;
    }

    private class REAL {
        // TODO tune-drive: PathPlanner/AutoDrive translation PID on robot.
        private static final double TRANSLATION_PID_P = 0.0;
        private static final double TRANSLATION_PID_D = 0.0;
        // TODO tune-drive: PathPlanner/AutoDrive rotation PID on robot.
        private static final double ROTATION_PID_P = 0.0;
        private static final double ROTATION_PID_D = 0.0;
        // TODO tune-drive: AutoDrive trapezoid limits (m/s and m/s^2).
        private static final double AUTO_DRIVE_MAX_VEL = 0.5;
        private static final double AUTO_DRIVE_MAX_ACCEL = 1.0;
    }

    private class AUTOAIM_ROTATION{
        private class SIMULATION {
            // TODO tune-drive: auto-aim PID for simulation.
            private static final double ROTATION_PID_P = 15.0;
            private static final double ROTATION_PID_D = 0.0;
        }

        private class REAL{
            // TODO tune-drive: auto-aim PID on robot.
            private static final double ROTATION_PID_P = 5.0;
            private static final double ROTATION_PID_D = 0.0;
        }
    }

    // TODO tune-drive: AutoDrive completion tolerance.
    public static final Pose2d TOLORANCE_AUTO_DRIVE = new Pose2d(
        0.02,
        0.02,
        Rotation2d.fromDegrees(1)
    );
    //#endregion

    //#region Pathplanner & AutoDrive
    public static final double TRANSLATION_PID_P = 
        Utils.isSimulation() 
            ? SIMULATION.TRANSLATION_PID_P 
            : REAL.TRANSLATION_PID_P;

    public static final double TRANSLATION_PID_D = 
        Utils.isSimulation() 
            ? SIMULATION.TRANSLATION_PID_D 
            : REAL.TRANSLATION_PID_D;

    public static final double ROTATION_PID_P = 
        Utils.isSimulation() 
            ? SIMULATION.ROTATION_PID_P 
            : REAL.ROTATION_PID_P;

    public static final double ROTATION_PID_D =
        Utils.isSimulation()
            ? SIMULATION.ROTATION_PID_D
            : REAL.ROTATION_PID_D;

    public static final double AUTO_DRIVE_MAX_VEL =
        Utils.isSimulation()
            ? SIMULATION.AUTO_DRIVE_MAX_VEL
            : REAL.AUTO_DRIVE_MAX_VEL;

    public static final double AUTO_DRIVE_MAX_ACCEL =
        Utils.isSimulation()
            ? SIMULATION.AUTO_DRIVE_MAX_ACCEL
            : REAL.AUTO_DRIVE_MAX_ACCEL;
    //#endregion

    //#region AutoAim
    // Feedforward is computed in AimCalc and should not be tuned here.
    public static final double AUTOAIM_ROTATION_PID_P = 
        Utils.isSimulation() 
            ? AUTOAIM_ROTATION.SIMULATION.ROTATION_PID_P 
            : AUTOAIM_ROTATION.REAL.ROTATION_PID_P;

    public static final double AUTOAIM_ROTATION_PID_D =
        Utils.isSimulation()
            ? AUTOAIM_ROTATION.SIMULATION.ROTATION_PID_D
            : AUTOAIM_ROTATION.REAL.ROTATION_PID_D;
    //#endregion
}
