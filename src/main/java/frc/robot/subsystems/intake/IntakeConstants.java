package frc.robot.subsystems.intake;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeConstants {
    // state space stuff
    // will be replaced with actual kV/kA later
    public static final DCMotor ARM_GEARBOX = DCMotor.getKrakenX60(2);
    public static final double ARM_MOI = SingleJointedArmSim.estimateMOI(
        0.26035,
        6
    );
    public static final LinearSystem<N2, N1, N2> ARM_STATE_SPACE_TEMP = LinearSystemId.createDCMotorSystem(
        IntakeConstants.ARM_GEARBOX,
        IntakeConstants.ARM_MOI,
        HAL.ARM_GEARING
    );
    public static final LinearSystem<N2, N1, N2> ARM_STATE_SPACE = ARM_STATE_SPACE_TEMP;

    public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60(1);
    public static final LinearSystem<N2, N1, N2> ROLLER_STATE_SPACE = LinearSystemId.createDCMotorSystem(0.1, 0.01);

    public class HAL{
        public static final double ARM_GEARING = 23.0 * 1.2587890624;
        public static final double ROLLER_GEARING = 1.0; // TODO

        public static final boolean ROLLER_INVERT = true;
        public static final boolean ARM_INVERT = true;

        public static final double ARM_PID_P = 0;
        public static final double ARM_PID_I = 0;
        public static final double ARM_PID_D = 0;

        public static final double ROLLER_PID_P = 0;
        public static final double ROLLER_PID_I = 0;
        public static final double ROLLER_PID_D = 0;

    }

    public class CAD{
        public static final double ORIGIN_OFFSET_X = 0.324;
        public static final double ORIGIN_OFFSET_Y = -0.349; 
        public static final double ORIGIN_OFFSET_Z = 0.194;

        public static final double PITCH_OFFSET = Math.toRadians(-50);
    }
}
