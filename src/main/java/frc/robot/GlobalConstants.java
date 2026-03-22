package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FlippingUtil.FieldSymmetry;

public class GlobalConstants {
    /*
     * COMPETITION MODE:
     * Visualization for subsytems including autoaim
     * Only load autos with prefix "comp_"
     * Publish AKit Logging data
     */
    public static final Boolean COMP = false;
    
    /*
     * SYSID: (commands will be added to autoChooser)
     * Swerve uses CTRE log
     * FLYWHEELS uses AdvantageKit
     */
    public static final Boolean SYS_ID_SWERVE = false;
    public static final String CTRE_LOG_PATH = "/media/sda1/ctre-logs/";

    public static final Boolean SYS_ID_FLYWHEELS = false;

    public static final CANBus BUS = new CANBus("rio");

    public class FIELD_CONSTANTS {
        public static FieldSymmetry symmetryType = FieldSymmetry.kRotational;
        public static final double FIELD_WIDTH = 8.07;
        public static final double FIELD_LENGTH = 16.54;

        public static final double BLUE_ALLIANCE_ZONE_X = 4.0284;
        public static final double RED_ALLIANCE_ZONE_X = FIELD_LENGTH - BLUE_ALLIANCE_ZONE_X;

        // AutoAim
        public static final Translation2d BLUE_HUB = new Translation2d(
            BLUE_ALLIANCE_ZONE_X + 0.5969, // 23.5 in
            FIELD_WIDTH / 2
        );

        public static final Translation2d RED_HUB = FlippingUtil.flipFieldPosition(BLUE_HUB);

        // Passing
        public static final Translation2d TOP_LEFT = new Translation2d(2.5, 6);
        public static final Translation2d TOP_RIGHT = new Translation2d(FIELD_LENGTH - 2.5, 6);
        public static final Translation2d BOTTOM_LEFT = new Translation2d(2.5, 2);
        public static final Translation2d BOTTOM_RIGHT = new Translation2d(FIELD_LENGTH - 2.5, 2);

        // Climbing
        // TODO: Tune for actual field
        private static final Pose2d BLUE_UP = new Pose2d(0, 0, Rotation2d.k180deg);
        private static final Pose2d BLUE_DOWN = new Pose2d(0, 0, Rotation2d.kZero);
        // use flipping utils if works
        private static final Pose2d RED_UP = new Pose2d(0, 0, Rotation2d.kZero);
        private static final Pose2d RED_DOWN = new Pose2d(15.0, 4.5, Rotation2d.k180deg);

        public static final List<Pose2d> CLIMB_POSITIONS = Arrays.asList(
            BLUE_UP, BLUE_DOWN, RED_UP, RED_DOWN
        );
    }
}
