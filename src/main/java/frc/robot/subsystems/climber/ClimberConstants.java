package frc.robot.subsystems.climber;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class ClimberConstants {
    public static final DCMotor CLIMBER_GEARBOX = DCMotor.getNeoVortex(1);
    public static final LinearSystem<N2, N1, N2> CLIMBER_STATE_SPACE = LinearSystemId.createDCMotorSystem(0.1, 0.01);
    
    public class HAL{
        public static final double CLIMBER_GEARING = 1.0;

        public static final boolean CLIMBER_INVERT = false;
    }

    public class TUNING{
        public static final int CLIMBER_STALL_LIMIT = 60;
        public static final int CLIMBER_FREE_LIMIT = 50;

        // Spark relative encoder position is left in motor rotations so these
        // soft limits can be tuned directly from the encoder reading.
        public static final double CLIMBER_MIN_POSITION = 0.0;
        public static final double CLIMBER_MAX_POSITION = 264.0;
    }
}
