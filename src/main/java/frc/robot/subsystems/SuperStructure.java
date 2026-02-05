package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class SuperStructure extends SubsystemBase{
    public final ShooterIO shooterIO; 
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer shooterConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hoodDisconnected, shooterDisconnected;

    public final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
    private final Debouncer armConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert armDisconnected, rollerDisconnected;

    public final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();
    private final Debouncer hopperConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hopperDisconnected;

    public SuperStructure(ShooterIO shooterIO, IntakeIO intakeIO, HopperIO hopperIO){
        this.shooterIO = shooterIO;
        this.intakeIO = intakeIO;
        this.hopperIO = hopperIO;

        hoodDisconnected = new Alert("hood motor disconnected!", Alert.AlertType.kWarning);
        shooterDisconnected = new Alert("shooter motor disconnected!", Alert.AlertType.kWarning);
        armDisconnected = new Alert("arm motor disconnected!", Alert.AlertType.kWarning);
        rollerDisconnected = new Alert("roller motor disconnected!", Alert.AlertType.kWarning);
        hopperDisconnected = new Alert("hopper motor disconnected!", Alert.AlertType.kWarning);
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
        armDisconnected.set(!armConnectedDebouncer.calculate(intakeInputs.armConnected));
        rollerDisconnected.set(!rollerConnectedDebouncer.calculate(intakeInputs.rollerConnected));
        hopperDisconnected.set(!hopperConnectedDebouncer.calculate(hopperInputs.hopperConnected));
    }

    // TODO test to find actual values & implement calculation later
    public Command runIntake(double userInput){
        return Commands.parallel(
            intakeIO.runArm(userInput),
            intakeIO.runRoller(10)
        ).finallyDo(
            (interrupted) -> intakeIO.stopMotors()
        ).withName("SuperStructure.runIntake");
    }

    // TODO maybe move intake up and down?
    public Command runShooter(){
        return Commands.parallel(
            shooterIO.runShooter(1000),
            shooterIO.runHood(Math.toRadians(30)),
            Commands.waitUntil(shooterIO::isStable).andThen(hopperIO.runHopper(100))
        ).finallyDo(
            (interrupted) -> {
                shooterIO.stopMotors();
                hopperIO.stopMotors();
            }
        ).withName("SuperStructure.runShooter");
    }
}
