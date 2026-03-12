package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shooter.ShooterConstants.*;
import frc.robot.utils.calc.AimCalc.ShootingParams;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.GlobalConstants;

public class ShooterIOKraken implements ShooterIO{
    private final TalonFX hoodMotor, shooter, secondaryShooter;

    // Status Signals
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
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
    private final MotionMagicVoltage hoodPositionVoltage = new MotionMagicVoltage(0.0)
        .withUpdateFreqHz(50.0);
    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0.0)
        .withUpdateFreqHz(50.0);
    private final Follower followerRequest = new Follower(
        41,
        HAL.SECONDARY_SHOOTER_INVERT 
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned
    ).withUpdateFreqHz(50.0);
    
    // Cached last commanded targets, used by isStable() feed gating and applyOutputs().
    private double hoodSetpointRad = 0.0;
    private double shooterSetpointRadPerSec = 0.0;
    private Boolean shooterCloseLoopEnabled = false, hoodCloseLoopEnabled = false;

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
        
        this.hoodConfig.Slot0.kS = TUNING.HOOD_KS;
        this.hoodConfig.Slot0.kV = TUNING.HOOD_KV;
        this.hoodConfig.Slot0.kG = TUNING.HOOD_KG;
        
        // this should be arm, just because the difference is not that big
        this.hoodConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        this.hoodConfig.Slot0.GravityArmPositionOffset = TUNING.HOOD_GRAVITY_ANGLE_OFFSET_RAD;

        this.hoodConfig.MotionMagic.MotionMagicCruiseVelocity = TUNING.HOOD_MAX_VEL;  
        this.hoodConfig.MotionMagic.MotionMagicAcceleration = TUNING.HOOD_MAX_ACCEL; 
        
        this.hoodConfig.MotorOutput.Inverted =
            HAL.HOOD_INVERT
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        this.hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.hoodConfig.Feedback.SensorToMechanismRatio = HAL.HOOD_GEARING;

        this.shooterConfig.Slot0.kP = TUNING.SHOOTER_PID_P;
        this.shooterConfig.Slot0.kI = TUNING.SHOOTER_PID_I;
        this.shooterConfig.Slot0.kD = TUNING.SHOOTER_PID_D;
        this.shooterConfig.Slot0.kV = TUNING.SHOOTER_KV;
        this.shooterConfig.MotorOutput.Inverted =
            HAL.SHOOTER_INVERT
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        this.shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        this.shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.shooterConfig.Feedback.SensorToMechanismRatio = HAL.SHOOTER_GEARING;

        this.hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.hoodConfig.CurrentLimits.StatorCurrentLimit = 80;
        this.hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.hoodConfig.CurrentLimits.SupplyCurrentLimit = 30;

        this.shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.shooterConfig.CurrentLimits.StatorCurrentLimit = 120;
        this.shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;

        // Status Signals
        this.hoodPosition = hoodMotor.getPosition();
        this.hoodVelocity = hoodMotor.getVelocity();
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
            this.hoodVelocity,
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
            this.tempSecondaryShooter,
            this.shooter.getControlMode()
        );

        shooter.optimizeBusUtilization(1.0);
        secondaryShooter.optimizeBusUtilization(1.0);
        hoodMotor.optimizeBusUtilization(1.0);

        hoodMotor.getConfigurator().apply(hoodConfig);
        shooter.getConfigurator().apply(shooterConfig);
        secondaryShooter.getConfigurator().apply(shooterConfig);

        hoodMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.hoodConnected = BaseStatusSignal.refreshAll(
            this.hoodPosition,
            this.hoodVelocity,
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
        inputs.hoodVelocity = this.hoodVelocity.getValueAsDouble() * Math.PI * 2;
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
    public void applyOutputs() {
        if(hoodCloseLoopEnabled){
            hoodMotor.setControl(
                hoodPositionVoltage.withPosition(hoodSetpointRad / (Math.PI * 2))
            );
        }
        

        if (shooterCloseLoopEnabled) {
            shooter.setControl(
                shooterVelocityVoltage.withVelocity(shooterSetpointRadPerSec / (Math.PI * 2))
            );

            secondaryShooter.setControl(followerRequest);
        }
    }

    @Override
    public void runHood(Supplier<ShootingParams> params) {
        hoodSetpointRad = params.get().pitchRads;
        hoodCloseLoopEnabled = true;
    }

    @Override
    public void runShooter(Supplier<ShootingParams> params) {
        shooterSetpointRadPerSec = params.get().velocityRadsPerSec;
        shooterCloseLoopEnabled = true;
    }

    @Override
    public void runHoodVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void runShooterVoltage(double voltage) {
        shooter.setVoltage(voltage);
        secondaryShooter.setVoltage(voltage);
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
    public void stopHood() {
        hoodCloseLoopEnabled = false;
        hoodMotor.stopMotor();
    }

    @Override
    public void stopShooter() {
        shooterCloseLoopEnabled = false;
        shooter.stopMotor();
        secondaryShooter.stopMotor();
    }
}
