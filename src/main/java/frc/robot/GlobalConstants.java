package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;

public class GlobalConstants {
    // Comp
    public static final Boolean COMP = false;
    
    // System Identification
    public static final Boolean SYS_ID = false;
    public static final String CTRE_LOG_PATH = "/media/sda1/ctre-logs/";

    public static final CANBus CARNIVORE = new CANBus("*");

    // Driver Preferences
    // it's not tanh(x) just because 
    // upper part of it looks like tanh(x)
    public static final String LEFT_AXIS_CONFIG = "tanh";
    public static final String RIGHT_AXIS_CONFIG = "tanhRightAxis";

    public class FIELD_CONSTANTS {
        public static final double FIELD_WIDTH = 8.07;
        public static final double FIELD_LENGTH = 16.54;

        public static final double BLUE_ALLIANCE_ZONE_X = 4.0284;
        public static final double RED_ALLIANCE_ZONE_X = FIELD_LENGTH - BLUE_ALLIANCE_ZONE_X;

        public static final Translation2d BLUE_HUB = new Translation2d(
            BLUE_ALLIANCE_ZONE_X + 0.5969, // 23.5 in
            FIELD_WIDTH / 2
        );

        public static final Translation2d RED_HUB = new Translation2d(
            RED_ALLIANCE_ZONE_X - 0.5969,  // 23.5 in
            FIELD_WIDTH / 2
        );

        // Positions to aim when Passing
        public static final Translation2d TOP_LEFT = new Translation2d(2.5, 6);
        public static final Translation2d TOP_RIGHT = new Translation2d(FIELD_LENGTH - 2.5, 6);
        public static final Translation2d BOTTOM_LEFT = new Translation2d(2.5, 2);
        public static final Translation2d BOTTOM_RIGHT = new Translation2d(FIELD_LENGTH - 2.5, 2);
    }
}
