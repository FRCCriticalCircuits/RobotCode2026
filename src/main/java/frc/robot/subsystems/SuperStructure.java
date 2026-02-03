package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class SuperStructure extends SubsystemBase{
    private final ShooterIO shooterIO; 
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer shooterConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hoodDisconnected, shooterDisconnected;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
    private final Debouncer pivotConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert pivotDisconnected, rollerDisconnected;

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();
    private final Debouncer hopperConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hopperDisconnected;

    public SuperStructure(ShooterIO shooterIO, IntakeIO intakeIO, HopperIO hopperIO){
        this.shooterIO = shooterIO;
        this.intakeIO = intakeIO;
        this.hopperIO = hopperIO;

        hoodDisconnected = new Alert("hood motor disconnected!", Alert.AlertType.kWarning);
        shooterDisconnected = new Alert("shooter motor disconnected!", Alert.AlertType.kWarning);
        pivotDisconnected = new Alert("pivot motor disconnected!", Alert.AlertType.kWarning);
        rollerDisconnected = new Alert("roller motor disconnected!", Alert.AlertType.kWarning);
        hopperDisconnected = new Alert("hopper motor disconnected!", Alert.AlertType.kWarning);

        // motor.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        intakeIO.updateInputs(intakeInputs);
        hopperIO.updateInputs(hopperInputs);

        Logger.processInputs("ShooterIO", shooterInputs);
        Logger.processInputs("IntakeIO", intakeInputs);
        Logger.processInputs("HopperIO", hopperInputs);
        
        hoodDisconnected.set(!hoodConnectedDebouncer.calculate(shooterInputs.hoodConnected));
        shooterDisconnected.set(!shooterConnectedDebouncer.calculate(shooterInputs.shooterConnected));
        pivotDisconnected.set(!pivotConnectedDebouncer.calculate(intakeInputs.pivotConnected));
        rollerDisconnected.set(!rollerConnectedDebouncer.calculate(intakeInputs.rollerConnected));
        hopperDisconnected.set(!hopperConnectedDebouncer.calculate(hopperInputs.hopperConnected));
    }

    public void setPivot(double positionRad){
        intakeIO.runPivot(positionRad);
    }

    public void setIntakeRoller(double velocity){
        intakeIO.runRoller(velocity);
    }

    public void setHopper(double velocity){
        hopperIO.runHopper(velocity);
    }
}
