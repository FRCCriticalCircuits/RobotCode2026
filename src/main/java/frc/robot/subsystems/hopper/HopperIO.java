package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public class HopperIOInputs {
        public double hopperVelocity = 0;

        public double appliedVoltsHopper = 0;
        public double supplyCurrentHopper = 0;
        public double torqueCurrentHopper = 0;

        public double tempHopper = 0;

        public boolean hopperConnected = false;
    }

    default void updateInputs(HopperIOInputs inputs) {}
    
    default void runHopper(double velocity) {}
}
