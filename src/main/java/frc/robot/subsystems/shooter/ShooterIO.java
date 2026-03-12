package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.calc.AimCalc.ShootingParams;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double hoodPosition = 0;
        public double hoodVelocity = 0;
        public double shooterPosition = 0;
        public double shooterVelocity = 0;

        public double appliedVoltsHood = 0;
        public double supplyCurrentHood = 0;
        public double torqueCurrentHood = 0;

        public double appliedVoltsShooter = 0;
        public double supplyCurrentShooter = 0;
        public double torqueCurrentShooter = 0;

        public double tempHood = 0;
        public double tempShooter = 0;
        public double tempSecondaryShooter = 0;

        public boolean hoodConnected = false;
        public boolean shooterConnected = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {}
    default void applyOutputs() {}
    
    default void runHood(Supplier<ShootingParams> params) {}
    default void runShooter(Supplier<ShootingParams> params) {}
    
    default void runHoodVoltage(double voltage) {}
    default void runShooterVoltage(double voltage) {}

    default Boolean isStable() {return false;}

    default void stopHood() {}
    default void stopShooter() {}
}
