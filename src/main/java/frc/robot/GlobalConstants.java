package frc.robot;

import com.ctre.phoenix6.CANBus;

public class GlobalConstants {
    // Comp
    public static final Boolean COMP = false;
    
    // System Identification
    public static final Boolean SYS_ID = false;
    public static final String CTRE_LOG_PATH = "/media/sda1/ctre-logs/";

    public static final CANBus CARNIVORE = new CANBus("*");

    // Driver Preferences
    // not tanh(x) just because upper part of it looks like tanh(x)
    public static final String LEFT_AXIS_CONFIG = "tanh";
    public static final String RIGHT_AXIS_CONFIG = "tanhRightAxis";
}
