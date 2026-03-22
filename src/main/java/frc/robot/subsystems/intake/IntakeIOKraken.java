package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.intake.IntakeConstants.*;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.GlobalConstants;

public class IntakeIOKraken implements IntakeIO{
    private final TalonFX armMotor, rollerMotor, secondaryArmMotor;
    
    // Status Signals
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> appliedVoltsArm;
    private final StatusSignal<Current> supplyCurrentArm;
    private final StatusSignal<Current> torqueCurrentArm;
    private final StatusSignal<Voltage> appliedVoltsRoller;
    private final StatusSignal<Current> supplyCurrentRoller;
    private final StatusSignal<Current> torqueCurrentRoller;
    private final StatusSignal<Temperature> tempArm;
    private final StatusSignal<Temperature> tempSecondaryArm;
    private final StatusSignal<Temperature> tempRoller;

    // Configuration
    private final TalonFXConfiguration armConfig, rollerConfig;

    // Control Requests
    private final PositionVoltage armPostionVoltage = new PositionVoltage(0.0)
        .withUpdateFreqHz(50.0);
    private final VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0.0)
        .withUpdateFreqHz(50.0);
        private final VoltageOut vout = new VoltageOut(12);

    private final Follower followerRequest = new Follower(
        30,
        MotorAlignmentValue.Opposed
    ).withUpdateFreqHz(50.0);

    // Cached Desired States
    private double desiredArmPositionRot = 0.0;      // rotations
    private double desiredRollerVelocityRps = 0.0;   // rotations per second
    private boolean rollerClosedLoopEnabled = false, armClosedLoopEnabled = false;

    public IntakeIOKraken(){
        this.armMotor = new TalonFX(30, GlobalConstants.BUS);
        this.secondaryArmMotor = new TalonFX(31, GlobalConstants.BUS);
        this.rollerMotor = new TalonFX(32, GlobalConstants.BUS);

        // Configuration
        this.armConfig = new TalonFXConfiguration();
        this.rollerConfig = new TalonFXConfiguration();
    
        this.armConfig.Slot0.kP = TUNING.ARM_PID_P;
        this.armConfig.Slot0.kI = 0; // not used for FRC
        this.armConfig.Slot0.kD = TUNING.ARM_PID_D;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.armConfig.Slot0.kS = 0;
        this.armConfig.Slot0.kV = 0;
        this.armConfig.Slot0.kG = 0;

        // this.armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // this.armConfig.Slot0.GravityArmPositionOffset = TUNING.ARM_GRAVITY_ANGLE_OFFSET_RAD;

        this.armConfig.MotionMagic.MotionMagicCruiseVelocity = TUNING.ARM_MAX_VEL;  
        this.armConfig.MotionMagic.MotionMagicAcceleration = TUNING.ARM_MAX_ACCEL; 

        this.armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.armConfig.Feedback.SensorToMechanismRatio = HAL.ARM_GEARING;

        this.rollerConfig.Slot0.kP = TUNING.ROLLER_PID_P;
        this.rollerConfig.Slot0.kI = TUNING.ROLLER_PID_I;
        this.rollerConfig.Slot0.kD = TUNING.ROLLER_PID_D;
        this.rollerConfig.Slot0.kS = TUNING.ROLLER_KS;
        this.rollerConfig.Slot0.kV = TUNING.ROLLER_KV;

        this.rollerConfig.MotorOutput.Inverted =
            HAL.ROLLER_INVERT
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        this.rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        this.rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.rollerConfig.Feedback.SensorToMechanismRatio = HAL.ROLLER_GEARING;
        
        this.armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.armConfig.CurrentLimits.StatorCurrentLimit = 80;
        this.armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.armConfig.CurrentLimits.SupplyCurrentLimit = 30;

        this.rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
        this.rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.rollerConfig.CurrentLimits.SupplyCurrentLimit = 20;

        // Status Signals
        this.armPosition = armMotor.getPosition();
        this.armVelocity = armMotor.getVelocity();
        this.rollerPosition = rollerMotor.getPosition();
        this.rollerVelocity = rollerMotor.getVelocity();

        this.appliedVoltsArm = armMotor.getMotorVoltage();
        this.supplyCurrentArm = armMotor.getSupplyCurrent();
        this.torqueCurrentArm = armMotor.getTorqueCurrent();

        this.appliedVoltsRoller = rollerMotor.getMotorVoltage();
        this.supplyCurrentRoller = rollerMotor.getSupplyCurrent();
        this.torqueCurrentRoller = rollerMotor.getTorqueCurrent();
        
        this.tempArm = armMotor.getDeviceTemp();
        this.tempSecondaryArm = secondaryArmMotor.getDeviceTemp();
        this.tempRoller = rollerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, 
            this.armPosition,
            this.armVelocity,
            this.rollerPosition,
            this.rollerVelocity,
            this.appliedVoltsArm,
            this.supplyCurrentArm,
            this.torqueCurrentArm,
            this.appliedVoltsRoller,
            this.supplyCurrentRoller,
            this.torqueCurrentRoller,
            this.tempArm,
            this.tempSecondaryArm,
            this.tempRoller,
            this.armMotor.getControlMode()
        );
        
        armMotor.optimizeBusUtilization(1.0);
        secondaryArmMotor.optimizeBusUtilization(1.0);
        rollerMotor.optimizeBusUtilization(1.0);

        armMotor.getConfigurator().apply(armConfig);
        secondaryArmMotor.getConfigurator().apply(armConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);

        // armMotor.setPosition(HAL.DEFAULT_ARM_POSITION_ROT);
        armMotor.setPosition(0);
        // doesnt matter for the second motor cuz we're using follower control
        // secondaryArmMotor.setPosition(HAL.DEFAULT_ARM_POSITION_ROT);
        secondaryArmMotor.setPosition(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.rollerConnected = BaseStatusSignal.refreshAll(
            this.rollerPosition,
            this.rollerVelocity,
            this.appliedVoltsRoller,
            this.supplyCurrentRoller,
            this.torqueCurrentRoller,
            this.tempRoller
        ).isOK();

        inputs.armConnected = BaseStatusSignal.refreshAll(
            this.armPosition,
            this.armVelocity,
            this.appliedVoltsArm,
            this.supplyCurrentArm,
            this.torqueCurrentArm,
            this.tempArm,
            this.tempSecondaryArm
        ).isOK();

        inputs.rollerPosition = this.rollerPosition.getValueAsDouble() * Math.PI * 2;
        inputs.rollerVelocity = this.rollerVelocity.getValueAsDouble() * Math.PI * 2;
        inputs.armPosition = this.armPosition.getValueAsDouble() * Math.PI * 2;
        inputs.armVelocity = this.armVelocity.getValueAsDouble() * Math.PI * 2;

        inputs.appliedVoltsRoller = this.appliedVoltsRoller.getValueAsDouble();
        inputs.supplyCurrentRoller = this.supplyCurrentRoller.getValueAsDouble();
        inputs.torqueCurrentRoller = this.torqueCurrentRoller.getValueAsDouble();

        inputs.appliedVoltsArm = this.appliedVoltsArm.getValueAsDouble();
        inputs.supplyCurrentArm = this.supplyCurrentArm.getValueAsDouble();
        inputs.torqueCurrentArm = this.torqueCurrentArm.getValueAsDouble();
        
        inputs.tempArm = this.tempArm.getValueAsDouble();
        inputs.tempSecondaryArm = this.tempSecondaryArm.getValueAsDouble();
        inputs.tempRoller = this.tempRoller.getValueAsDouble();
    }

    @Override
    public void applyOutputs() {
        if(armClosedLoopEnabled) {
            armMotor.setControl(
                armPostionVoltage.withPosition(desiredArmPositionRot)
            );

            secondaryArmMotor.setControl(
                followerRequest
            );
        }

        if (rollerClosedLoopEnabled) {
            rollerMotor.setControl(
                rollerVelocityVoltage.withVelocity(desiredRollerVelocityRps)
            );
        }
    }

    @Override
    public void setArmPosition(Angle angle) {
        desiredArmPositionRot = angle.in(Rotations);
        armClosedLoopEnabled = true;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        desiredRollerVelocityRps = velocityRadPerSec / (Math.PI * 2.0);
        rollerClosedLoopEnabled = true;
    }

    @Override
    public void runArmVoltage(double voltage) {
        // DISABLED
    }

    @Override
    public void runRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void stopArm() {
        armClosedLoopEnabled = false;
        armMotor.stopMotor();
        secondaryArmMotor.stopMotor();
    }

    @Override
    public void stopRoller() {
        rollerClosedLoopEnabled = false;
        rollerMotor.stopMotor();
    }
}
