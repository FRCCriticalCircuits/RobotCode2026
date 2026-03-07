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
    private static final double ARM_MOI = SingleJointedArmSim.estimateMOI(
        0.26035,
        6
    );
    private static final LinearSystem<N2, N1, N2> ARM_STATE_SPACE_TEMP = LinearSystemId.createDCMotorSystem(
        IntakeConstants.ARM_GEARBOX,
        IntakeConstants.ARM_MOI,
        HAL.ARM_GEARING
    );
    public static final LinearSystem<N2, N1, N2> ARM_STATE_SPACE = ARM_STATE_SPACE_TEMP;

    public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60(1);
    public static final LinearSystem<N2, N1, N2> ROLLER_STATE_SPACE = LinearSystemId.createDCMotorSystem(0.1, 0.01);

    public class HAL{
        // Keep gearbox and linkage conversion split so measurement updates stay clear.
        public static final double ARM_GEARING = 23.0 * 1.2587890624;
        // TODO tune-hardware: update to match measured roller-to-mechanism ratio.
        public static final double ROLLER_GEARING = 1.0;
        // TODO tune-hardware: update to match what's required by CTRE, horizontal at position 0
        public static final double DEFAULT_ARM_POSITION_ROT = -1.0;

        public static final boolean ARM_INVERT = false;
        public static final boolean SECONDARY_ARM_INVERT = true;
        public static final boolean ROLLER_INVERT = true;
    }

    public class TUNING{
        /*
            // tune Current Settings in IntakeIOKrakne.java if needed

            // Hood torque current caps used in ARMs Kraken configuration.
            public static final double ARM_PEAK_FORWARD_TORQUE_CURRENT = 30.0;
            public static final double ARM_PEAK_REVERSE_TORQUE_CURRENT = -30.0;

            // Hood torque current caps used in ROLLER Kraken configuration.
            public static final double ROLLER_PEAK_FORWARD_TORQUE_CURRENT = 30.0;
            public static final double ROLLER_PEAK_REVERSE_TORQUE_CURRENT = -30.0;
        */

        /**
         * with MotionMagic Control
         * kP : the proportion for position error
         * kD : the proportion for velocity error
         * *_VEL_FF -> kV : voltage per velocity unit
         */
        public static final double ARM_PID_P = 1.0;
        public static final double ARM_PID_I = 0;
        public static final double ARM_PID_D = 0.05;

        public static final double ARM_VEL_FF = 0.15;
        // Optional gravity term for arm MotionMagic.
        public static final double ARM_GRAVITY_FF = 0.0;
        public static final double ARM_GRAVITY_ANGLE_OFFSET_RAD = 0.0;
        public static final double ARM_MAX_VEL = 1.0;
        public static final double ARM_MAX_ACCEL = 2.0;

        public static final double ROLLER_PID_P = 0.4;
        public static final double ROLLER_PID_I = 0;
        public static final double ROLLER_PID_D = 0;
        // Optional velocity feedforward for roller closed-loop velocity.
        public static final double ROLLER_VEL_FF = 0.1;
    }

    public class CAD{
        public static final double ORIGIN_OFFSET_X = 0.324;
        public static final double ORIGIN_OFFSET_Y = -0.349; 
        public static final double ORIGIN_OFFSET_Z = 0.194;

        public static final double PITCH_OFFSET = Math.toRadians(-5);
    }
}
