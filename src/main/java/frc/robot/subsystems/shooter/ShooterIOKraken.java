package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HAL;
import frc.robot.subsystems.shooter.ShooterConstants.TUNING;

public class ShooterIOKraken implements ShooterIO{
    private final TalonFX hoodMotor, shooter, secondaryShooter;

    // Status Signals
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Angle> shooterPosition;
    private final StatusSignal<AngularVelocity> shooterVelocity;
    private final StatusSignal<Voltage> appliedVoltsHood;
    private final StatusSignal<Current> supplyCurrentHood;
    private final StatusSignal<Current> torqueCurrentHood;
    private final StatusSignal<Voltage> appliedVoltsShooter;
    private final StatusSignal<Current> supplyCurrentShooter;
    private final StatusSignal<Current> torqueCurrentShooter;
    private final StatusSignal<Temperature> tempHood;
    private final StatusSignal<Temperature> tempShooter;
    private final StatusSignal<Temperature> tempSecondaryShooter;

    // Configuration
    private final TalonFXConfiguration hoodConfig, shooterConfig;

    // Control Requests
    private final PositionTorqueCurrentFOC hoodPositionFOC = new PositionTorqueCurrentFOC(0.0)
        .withUpdateFreqHz(0.0);
    private final VelocityTorqueCurrentFOC shooterVelocityFOC = new VelocityTorqueCurrentFOC(0.0)
        .withUpdateFreqHz(0.0);
    // Cached last commanded targets, used by isStable() feed gating.
    private double hoodSetpointRad = 0.0;
    private double shooterSetpointRadPerSec = 0.0;

    public ShooterIOKraken(){
        this.hoodMotor = new TalonFX(40, GlobalConstants.BUS);
        this.shooter = new TalonFX(41, GlobalConstants.BUS);
        this.secondaryShooter = new TalonFX(42, GlobalConstants.BUS);

        // Configuration
        this.hoodConfig = new TalonFXConfiguration();
        this.shooterConfig = new TalonFXConfiguration();
        
        this.hoodConfig.Slot0.kP = TUNING.HOOD_PID_P;
        this.hoodConfig.Slot0.kI = TUNING.HOOD_PID_I;
        this.hoodConfig.Slot0.kD = TUNING.HOOD_PID_D;
        
        this.hoodConfig.MotorOutput.Inverted =
            HAL.HOOD_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.hoodConfig.Feedback.RotorToSensorRatio = HAL.HOOD_GEARING;

        this.shooterConfig.Slot0.kP = TUNING.SHOOTER_PID_P;
        this.shooterConfig.Slot0.kI = TUNING.SHOOTER_PID_I;
        this.shooterConfig.Slot0.kD = TUNING.SHOOTER_PID_D;
        this.shooterConfig.MotorOutput.Inverted =
            HAL.SHOOTER_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        this.shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.shooterConfig.Feedback.RotorToSensorRatio = HAL.SHOOTER_GEARING;

        // Pull torque limits from constants so real+sim tuning stays centralized.
        this.hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            TUNING.HOOD_PEAK_FORWARD_TORQUE_CURRENT;
        this.hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            TUNING.HOOD_PEAK_REVERSE_TORQUE_CURRENT;

        this.shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            TUNING.HOOD_PEAK_FORWARD_TORQUE_CURRENT;
        this.shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            TUNING.SHOOTER_PEAK_REVERSE_TORQUE_CURRENT;

        // dont know if we need it these
        this.hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.hoodConfig.CurrentLimits.StatorCurrentLimit = 80;
        this.hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.hoodConfig.CurrentLimits.SupplyCurrentLimit = 30;

        this.shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.shooterConfig.CurrentLimits.StatorCurrentLimit = 140;
        this.shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // Status Signals
        this.hoodPosition = hoodMotor.getPosition();
        this.shooterPosition = shooter.getPosition();
        this.shooterVelocity = shooter.getVelocity();

        this.appliedVoltsHood = hoodMotor.getMotorVoltage();
        this.supplyCurrentHood = hoodMotor.getSupplyCurrent();
        this.torqueCurrentHood = hoodMotor.getTorqueCurrent();

        this.appliedVoltsShooter = shooter.getMotorVoltage();
        this.supplyCurrentShooter = shooter.getSupplyCurrent();
        this.torqueCurrentShooter = shooter.getTorqueCurrent();

        this.tempHood = hoodMotor.getDeviceTemp();
        this.tempShooter = shooter.getDeviceTemp();
        this.tempSecondaryShooter = secondaryShooter.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            this.hoodPosition,
            this.shooterPosition,
            this.shooterVelocity,
            this.appliedVoltsHood,
            this.supplyCurrentHood,
            this.torqueCurrentHood,
            this.appliedVoltsShooter,
            this.supplyCurrentShooter,
            this.torqueCurrentShooter,
            this.tempHood,
            this.tempShooter,
            this.tempSecondaryShooter
        );

        shooter.optimizeBusUtilization(1.0);
        secondaryShooter.optimizeBusUtilization(1.0);
        hoodMotor.optimizeBusUtilization(1.0);

        hoodMotor.getConfigurator().apply(hoodConfig);
        shooter.getConfigurator().apply(shooterConfig);
        secondaryShooter.getConfigurator().apply(shooterConfig);

        secondaryShooter.setControl(
            new Follower(
                shooter.getDeviceID(), 
                HAL.SECONDARY_SHOOTER_INVERT 
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned
            )
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hoodConnected = BaseStatusSignal.refreshAll(
            this.hoodPosition,
            this.appliedVoltsHood,
            this.supplyCurrentHood,
            this.torqueCurrentHood,
            this.tempHood
        ).isOK();

        inputs.shooterConnected = BaseStatusSignal.refreshAll(
            this.shooterVelocity,
            this.appliedVoltsShooter,
            this.supplyCurrentShooter,
            this.torqueCurrentShooter,
            this.tempShooter,
            this.tempSecondaryShooter
        ).isOK();

        inputs.hoodPosition = this.hoodPosition.getValueAsDouble() * Math.PI * 2;
        inputs.shooterPosition = this.shooterPosition.getValueAsDouble() * Math.PI * 2;
        inputs.shooterVelocity = this.shooterVelocity.getValueAsDouble() * Math.PI * 2;
        
        inputs.appliedVoltsHood = this.appliedVoltsHood.getValueAsDouble();
        inputs.supplyCurrentHood = this.supplyCurrentHood.getValueAsDouble();
        inputs.torqueCurrentHood = this.torqueCurrentHood.getValueAsDouble();
        
        inputs.appliedVoltsShooter = this.appliedVoltsShooter.getValueAsDouble();
        inputs.supplyCurrentShooter = this.supplyCurrentShooter.getValueAsDouble();
        inputs.torqueCurrentShooter = this.torqueCurrentShooter.getValueAsDouble();
        
        inputs.tempHood = this.tempHood.getValueAsDouble();
        inputs.tempShooter = this.tempShooter.getValueAsDouble();
        inputs.tempSecondaryShooter = this.tempSecondaryShooter.getValueAsDouble();
    }

    @Override
    public Command runHood(DoubleSupplier positionRad) {
        return Commands.run(
            () -> {
                // Track target locally so readiness logic can compare measured error.
                hoodSetpointRad = positionRad.getAsDouble();
                hoodMotor.setControl(
                    hoodPositionFOC.withPosition(hoodSetpointRad)
                );
            }
        ).withName("Shooter.runHoodPosition");
    }

    @Override
    public Command runShooter(double velocity) {
        return Commands.run(
            () -> {
                // Track target locally so readiness logic can compare measured error.
                shooterSetpointRadPerSec = velocity;
                shooter.setControl(
                    shooterVelocityFOC.withVelocity(shooterSetpointRadPerSec)
                );
            }
        ).withName("Shooter.runShooterVelocity");
    }

    @Override
    public void runShooterVoltage(double voltage) {
        shooter.setVoltage(voltage);
    }
    
    @Override
    public Boolean isStable() {
        // Convert CTRE rotations to radians, then check against tunable tolerances.
        double hoodErrorRad = Math.abs((hoodPosition.getValueAsDouble() * Math.PI * 2.0) - hoodSetpointRad);
        double shooterErrorRadPerSec =
            Math.abs((shooterVelocity.getValueAsDouble() * Math.PI * 2.0) - shooterSetpointRadPerSec);
        return hoodErrorRad <= TUNING.HOOD_STABLE_TOLERANCE_RAD
            && shooterErrorRadPerSec <= TUNING.SHOOTER_STABLE_TOLERANCE_RAD_PER_SEC;
    }

    @Override
    public void stopMotors() {
        hoodMotor.setControl(new NeutralOut());
        shooter.setControl(new NeutralOut());
    }
}
