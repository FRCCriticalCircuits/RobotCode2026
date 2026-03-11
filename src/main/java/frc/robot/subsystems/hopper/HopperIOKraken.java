package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.hopper.HopperConstants.TUNING;
import frc.robot.subsystems.hopper.HopperConstants.HAL;

public class HopperIOKraken implements HopperIO {
    private final TalonFX hopperMotor;
    
    // Status Signals
    private final StatusSignal<Angle> hopperPosition;
    private final StatusSignal<AngularVelocity> hopperVelocity;
    private final StatusSignal<Voltage> appliedVoltsHopper;
    private final StatusSignal<Current> supplyCurrentHopper;
    private final StatusSignal<Current> torqueCurrentHopper;
    private final StatusSignal<Temperature> tempHopper;

    // Configuration
    private final TalonFXConfiguration hopperConfig;

    // Control Requests
    private final VelocityVoltage hopperVelocityVoltage = new VelocityVoltage(0.0)
        .withUpdateFreqHz(50.0);

    // Cached Desired States
    private double desiredHopperVelocityRps = 0.0;   // rotations per second
    private boolean hopperClosedLoopEnabled = false;

    public HopperIOKraken() {
        this.hopperMotor = new TalonFX(50, GlobalConstants.BUS);

        // Configuration
        this.hopperConfig = new TalonFXConfiguration();

        this.hopperConfig.Slot0.kS = TUNING.HOPPER_KS;
        this.hopperConfig.Slot0.kV = TUNING.HOPPER_KV;

        this.hopperConfig.MotorOutput.Inverted =
            HAL.HOPPER_INVERT
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        this.hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.hopperConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.hopperConfig.Feedback.SensorToMechanismRatio = HAL.HOPPER_GEARING;

        this.hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.hopperConfig.CurrentLimits.StatorCurrentLimit = 40;
        this.hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.hopperConfig.CurrentLimits.SupplyCurrentLimit = 20;

        // Status Signals
        this.hopperPosition = hopperMotor.getPosition();
        this.hopperVelocity = hopperMotor.getVelocity();

        this.appliedVoltsHopper = hopperMotor.getMotorVoltage();
        this.supplyCurrentHopper = hopperMotor.getSupplyCurrent();
        this.torqueCurrentHopper = hopperMotor.getTorqueCurrent();
        
        this.tempHopper = hopperMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, 
            this.hopperPosition,
            this.hopperVelocity,
            this.appliedVoltsHopper,
            this.supplyCurrentHopper,
            this.torqueCurrentHopper,
            this.tempHopper
        );
        
        hopperMotor.optimizeBusUtilization(1.0);

        hopperMotor.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperConnected = BaseStatusSignal.refreshAll(
            this.hopperPosition,
            this.hopperVelocity,
            this.appliedVoltsHopper,
            this.supplyCurrentHopper,
            this.torqueCurrentHopper,
            this.tempHopper
        ).isOK();

        inputs.hopperPosition = this.hopperPosition.getValueAsDouble() * Math.PI * 2;
        inputs.hopperVelocity = this.hopperVelocity.getValueAsDouble() * Math.PI * 2;

        inputs.appliedVoltsHopper = this.appliedVoltsHopper.getValueAsDouble();
        inputs.supplyCurrentHopper = this.supplyCurrentHopper.getValueAsDouble();
        inputs.torqueCurrentHopper = this.torqueCurrentHopper.getValueAsDouble();

        inputs.tempHopper = this.tempHopper.getValueAsDouble();
    }

    @Override
    public void applyOutputs() {
        if (hopperClosedLoopEnabled) {
            hopperMotor.setControl(
                hopperVelocityVoltage.withVelocity(desiredHopperVelocityRps)
            );
        }
    }

    @Override
    public void runHopper(double velocityRadPerSec) {
        this.desiredHopperVelocityRps = velocityRadPerSec / (Math.PI * 2.0);
        this.hopperClosedLoopEnabled = true;
    }

    @Override
    public void runHopperVoltage(double voltage) {
        hopperMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        this.hopperClosedLoopEnabled = false;
        hopperMotor.stopMotor();
    }
}
