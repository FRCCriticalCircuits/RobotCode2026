package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double hoodPosition = 0;
        public double shooterVelocity = 0;

        public double appliedVoltsHood = 0;
        public double supplyCurrentHood = 0;
        public double torqueCurrentHood = 0;

        public double appliedVoltsShooter = 0;
        public double supplyCurrentShooter = 0;
        public double torqueCurrentShooter = 0;

        public double tempHood = 0;
        public double tempShooter = 0;

        public boolean hoodConnected = false;
        public boolean shooterConnected = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {}
    
    default void runHood(double positionRad) {}
    default void runShooter(double velocity) {}
}
