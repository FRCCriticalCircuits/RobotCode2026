package frc.robot.subsystems.hopper;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class HopperConstants {
    public static final DCMotor HOPPER_GEARBOX = DCMotor.getNEO(1);
    public static final LinearSystem<N2, N1, N2> HOPPER_STATE_SPACE = LinearSystemId.createDCMotorSystem(0.02, 0.0006);
    public static final double HOPPER_KS = 0.13;

    public class HAL{
        public static final double HOPPER_GEARING = 1.0;

        public static final boolean HOPPER_INVERT = true;
    }

    public class TUNING{
        public static final int CLIMBER_STALL_LIMIT = 30;
        public static final int CLIMBER_FREE_LIMIT = 20;

        public static final double HOPPER_PID_P = 0;
        public static final double HOPPER_PID_I = 0;
        public static final double HOPPER_PID_D = 0;
    }
}
