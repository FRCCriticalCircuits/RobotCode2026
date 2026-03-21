package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.ShooterConstants.*;
import frc.robot.utils.calc.AimCalc.ShootingParams;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim hood, shooter;

    private final PIDController hoodController = new PIDController(10, 0, 0);

    private double appliedVoltsHood = 0, appliedVoltsShooter = 0;
    private double hoodPositionRads = 0, shooterVelocityRadsPerSec = 0;
    private boolean shooterCloseLoopEnabled = false;

    public ShooterIOSim() {
        hood = new DCMotorSim(
            ShooterConstants.HOOD_STATE_SPACE,
            ShooterConstants.HOOD_GEARBOX
        );

        shooter = new DCMotorSim(
            ShooterConstants.SHOOTER_STATE_SPACE,
            ShooterConstants.SHOOTER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hoodPosition = hood.getAngularPositionRad();
        inputs.hoodVelocity = hood.getAngularVelocityRadPerSec();
        inputs.shooterPosition = shooter.getAngularPositionRad();
        inputs.shooterVelocity = shooter.getAngularVelocityRadPerSec();

        inputs.appliedVoltsHood = this.appliedVoltsHood;
        inputs.supplyCurrentHood = hood.getCurrentDrawAmps();

        inputs.appliedVoltsShooter = this.appliedVoltsShooter;
        inputs.supplyCurrentShooter = shooter.getCurrentDrawAmps();

        inputs.hoodConnected = true;
        inputs.shooterConnected = true;
    }

    @Override
    public void applyOutputs() {
        hood.update(0.02);
        shooter.update(0.02);

        this.appliedVoltsHood = MathUtil.clamp(
            hoodController.calculate(
                hood.getAngularPositionRad(),
                this.hoodPositionRads
            ),
            -12.0,
            12.0
        );
        hood.setInputVoltage(appliedVoltsHood);

        if(shooterCloseLoopEnabled){
            this.appliedVoltsShooter = MathUtil.clamp(
                // not the right way to handle kS but assume we dont need voltage request < 0.18v
                this.shooterVelocityRadsPerSec * 0.018 + 0.18, 
                -12.0,
                12.0
            );  
        }else{
            this.appliedVoltsShooter = 0.0;
        }

        feedShooterVoltage(appliedVoltsShooter);
    }

    /* Feed voltage into simulation state-spate, considering KS */
    private void feedShooterVoltage(double voltage){
        if(Math.abs(voltage) < ShooterConstants.SHOOTER_KS){
            voltage = 0.0;
        }
        shooter.setInputVoltage(voltage - Math.signum(voltage) * ShooterConstants.SHOOTER_KS);
    }

    @Override
    public void runHood(Supplier<ShootingParams> params) {
        this.hoodPositionRads = MathUtil.clamp(
            params.get().pitchRads,
            TUNING.HOOD_MIN_POSITION_RAD,
            TUNING.HOOD_MAX_POSITION_RAD
        );
    }

    @Override
    public void runShooter(Supplier<ShootingParams> params) {
        this.shooterVelocityRadsPerSec = params.get().velocityRadsPerSec;
        this.shooterCloseLoopEnabled = true;
    }

    @Override
    public void runShooterVoltage(double voltage) {
        this.appliedVoltsShooter = voltage;
        // will ignore `shooterStopped` variable
        feedShooterVoltage(voltage);
    }

    @Override
    public Boolean isStable() {
        // Match Kraken readiness behavior in sim so sequencing logic is testable.
        double hoodErrorRad = Math.abs(hood.getAngularPositionRad() - hoodPositionRads);
        double shooterErrorRadPerSec = Math.abs(shooter.getAngularVelocityRadPerSec() - shooterVelocityRadsPerSec);
        return hoodErrorRad <= TUNING.HOOD_STABLE_TOLERANCE_RAD
            && shooterErrorRadPerSec <= TUNING.SHOOTER_STABLE_TOLERANCE_RAD_PER_SEC;
    }

    @Override
    public void stopHood() {
        this.hoodPositionRads = MathUtil.clamp(
            TUNING.HOOD_STOW_POSITION_RAD,
            TUNING.HOOD_MIN_POSITION_RAD,
            TUNING.HOOD_MAX_POSITION_RAD
        );
    }

    @Override
    public void stopShooter() {
        this.shooterCloseLoopEnabled = false;
    }
}
