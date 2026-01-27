package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double hoodPosition = 0;
        public double shooterVelocity = 0;

        public boolean hoodConnected = false;
        public boolean shooterConnected = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {}
    
    default void runHood(double positionRad) {}
    default void runShooter(double velocity) {}
}
