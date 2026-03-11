package frc.robot.subsystems.hopper;

public class HopperConstants {
    public class HAL{
        public static final double HOPPER_GEARING = 1.0;

        public static final boolean HOPPER_INVERT = false;
    }

    public class TUNING{
        public static final double HOPPER_KS = 0.2;
        public static final double HOPPER_KV = 0.1;

        public static final int HOPPER_STALL_LIMIT = 30;
        public static final int HOPPER_FREE_LIMIT = 20;
    }
}
