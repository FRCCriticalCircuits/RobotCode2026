package frc.robot.subsystems.climber;

public class ClimberConstants {
    public class HAL{
        public static final double CLIMBER_GEARING = 1.0;

        public static final boolean CLIMBER_INVERT = false;
    }

    public class TUNING{
        public static final int CLIMBER_STALL_LIMIT = 60;
        public static final int CLIMBER_FREE_LIMIT = 50;

        public static final double CLIMBER_PID_P = 0;
        public static final double CLIMBER_PID_I = 0;
        public static final double CLIMBER_PID_D = 0;
        public static final double ENCODER_POSTION = 0;
    }
}
