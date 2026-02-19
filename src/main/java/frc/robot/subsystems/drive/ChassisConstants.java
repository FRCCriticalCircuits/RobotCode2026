package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ChassisConstants {
    //#region modifiable
    private class SIMULATION {
        private static final double TRANSLATION_PID_P = 5.0;
        private static final double TRANSLATION_PID_D = 0.0;
        private static final double ROTATION_PID_P = 15.0;
        private static final double ROTATION_PID_D = 0.0;
    }

    private class REAL {
        private static final double TRANSLATION_PID_P = 0.0;
        private static final double TRANSLATION_PID_D = 0.0;
        private static final double ROTATION_PID_P = 0.0;
        private static final double ROTATION_PID_D = 0.0;
    }

    private class AUTOAIM_ROTATION{
        private class SIMULATION {
            private static final double ROTATION_PID_P = 15.0;
            private static final double ROTATION_PID_D = 0.0;
        }

        private class REAL{
            private static final double ROTATION_PID_P = 5.0;
            private static final double ROTATION_PID_D = 0.0;
        }
    }

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
    //#endregion

    //#region AutoAim
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
