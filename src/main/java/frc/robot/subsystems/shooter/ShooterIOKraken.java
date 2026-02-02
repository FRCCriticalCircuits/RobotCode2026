package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.GlobalConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HAL;

public class ShooterIOKraken implements ShooterIO{
    private final TalonFX hoodMotor, shooterMotor;

    // Status Signals
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> shooterVelocity;
    private final StatusSignal<Voltage> appliedVoltsHood;
    private final StatusSignal<Current> supplyCurrentHood;
    private final StatusSignal<Current> torqueCurrentHood;
    private final StatusSignal<Voltage> appliedVoltsShooter;
    private final StatusSignal<Current> supplyCurrentShooter;
    private final StatusSignal<Current> torqueCurrentShooter;
    private final StatusSignal<Temperature> tempHood;
    private final StatusSignal<Temperature> tempShooter;

    // Configuration
    private final TalonFXConfiguration hoodConfig;
    private final TalonFXConfiguration shooterConfig;

    // Control Requests
    private final PositionTorqueCurrentFOC positionFOC = new PositionTorqueCurrentFOC(0)
        .withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public ShooterIOKraken(){
        this.hoodMotor = new TalonFX(41, GlobalConstants.CARNIVORE);
        this.shooterMotor = new TalonFX(42, GlobalConstants.CARNIVORE);

        // Configuration
        this.hoodConfig = new TalonFXConfiguration();
        this.shooterConfig = new TalonFXConfiguration();
        
        this.hoodConfig.Slot0.kP = HAL.HOOD_PID_P;
        this.hoodConfig.Slot0.kI = HAL.HOOD_PID_I;
        this.hoodConfig.Slot0.kD = HAL.HOOD_PID_D;
        this.hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30.0;
        this.hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -30.0;
        this.hoodConfig.MotorOutput.Inverted =
            HAL.HOOD_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.hoodConfig.Feedback.RotorToSensorRatio = HAL.HOOD_GEARING;

        this.shooterConfig.Slot0.kP = HAL.SHOOTER_PID_P;
        this.shooterConfig.Slot0.kI = HAL.SHOOTER_PID_I;
        this.shooterConfig.Slot0.kD = HAL.SHOOTER_PID_D;
        this.shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        this.shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        this.shooterConfig.MotorOutput.Inverted =
            HAL.SHOOTER_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        this.shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.shooterConfig.Feedback.RotorToSensorRatio = HAL.SHOOTER_GEARING;

        // Status Signals
        this.hoodPosition = hoodMotor.getPosition();
        this.shooterVelocity = shooterMotor.getVelocity();

        this.appliedVoltsHood = hoodMotor.getMotorVoltage();
        this.supplyCurrentHood = hoodMotor.getSupplyCurrent();
        this.torqueCurrentHood = hoodMotor.getTorqueCurrent();

        this.appliedVoltsShooter = shooterMotor.getMotorVoltage();
        this.supplyCurrentShooter = shooterMotor.getSupplyCurrent();
        this.torqueCurrentShooter = shooterMotor.getTorqueCurrent();

        this.tempHood = hoodMotor.getDeviceTemp();
        this.tempShooter = shooterMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            this.hoodPosition,
            this.shooterVelocity,
            this.appliedVoltsHood,
            this.supplyCurrentHood,
            this.torqueCurrentHood,
            this.appliedVoltsShooter,
            this.supplyCurrentShooter,
            this.torqueCurrentShooter,
            this.tempHood,
            this.tempShooter
        );

        shooterMotor.optimizeBusUtilization(1.0);
        hoodMotor.optimizeBusUtilization(1.0);
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
            this.tempShooter
        ).isOK();

        inputs.hoodPosition = this.hoodPosition.getValueAsDouble() * Math.PI * 2;
        inputs.shooterVelocity = this.shooterVelocity.getValueAsDouble() * Math.PI * 2;
        
        inputs.appliedVoltsHood = this.appliedVoltsHood.getValueAsDouble();
        inputs.supplyCurrentHood = this.supplyCurrentHood.getValueAsDouble();
        inputs.torqueCurrentHood = this.torqueCurrentHood.getValueAsDouble();
        inputs.appliedVoltsShooter = this.appliedVoltsShooter.getValueAsDouble();
        inputs.supplyCurrentShooter = this.supplyCurrentShooter.getValueAsDouble();
        inputs.torqueCurrentShooter = this.torqueCurrentShooter.getValueAsDouble();
        inputs.tempHood = this.tempHood.getValueAsDouble();
        inputs.tempShooter = this.tempShooter.getValueAsDouble();
    }

    @Override
    public void runHood(double positionRad) {
        hoodMotor.setControl(
            positionFOC.withPosition(positionRad)
        );
    }

    @Override
    public void runShooter(double velocity) {
        shooterMotor.setControl(
            velocityVoltage.withVelocity(velocity)
        );
    }
}
