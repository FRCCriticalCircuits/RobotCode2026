package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ChassisConstants {
    //#region modifiable
    public class SIMULATION {
        private static final double TRANSLATION_PID_P = 5.0;
        private static final double TRANSLATION_PID_D = 0.0;
        private static final double ROTATION_PID_P = 15.0;
        private static final double ROTATION_PID_D = 0.0;
    }

    public class REAL {
        private static final double TRANSLATION_PID_P = 5.0;
        private static final double TRANSLATION_PID_D = 0.0;
        private static final double ROTATION_PID_P = 2.0;
        private static final double ROTATION_PID_D = 0.0;
    }

    public static final Pose2d TOLORANCE_AUTO_DRIVE = new Pose2d(
        0.02,
        0.02,
        Rotation2d.fromDegrees(1)
    );
    //#endregion

    //#region computations
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
}
