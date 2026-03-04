// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PPDriveCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOVortex;
import frc.robot.subsystems.drive.Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveTelemetry;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionPhoton;
import frc.robot.utils.AimCalc;
import frc.robot.utils.axis.AxisConfigLoader;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();

    // Swerve
    private final Drive drivetrain = TunerConstants.createDrivetrain();
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();     
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // AutoAim
    private final Trigger autoAimTrigger = driverController.rightTrigger(0.15).debounce(0.02);
    
    private final AimCalc calculationUtil = new AimCalc(drivetrain);

    private final DriveCommand teleDrive = new DriveCommand(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> autoAimTrigger.getAsBoolean(),
        AxisConfigLoader.loadTable(GlobalConstants.LEFT_AXIS_CONFIG),
        AxisConfigLoader.loadTable(GlobalConstants.RIGHT_AXIS_CONFIG),
        () -> calculationUtil.getAimParams()
    );

    // need this separate command because
    // teleDrive cant work with pathplanner (NamedCommands)
    // it's the default command of drive subsystem
    private final PPDriveCommand pathplannerDrive = new PPDriveCommand(
        drivetrain,
        () -> calculationUtil.getAimParams()
    );

    // Vision
    private final VisionPhoton leftCam = new VisionPhoton("cameraLeft", VisionConstants.leftCam);
    private final VisionPhoton rightCam = new VisionPhoton("cameraRight", VisionConstants.rightCam);
    @SuppressWarnings("unused")
    private final Vision visionSubsystem = new Vision(drivetrain, leftCam, rightCam);

    // SuperStructure    
    private final ShooterIO shooterIO = Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOSim();
    private final IntakeIO intakeIO = Utils.isSimulation() ? new IntakeIOSim() : new IntakeIOSim();
    private final HopperIO hopperIO = Utils.isSimulation() ? new HopperIOSim() : new HopperIOSim();
    private final ClimberIO climberIO = Utils.isSimulation() ? new ClimberIOSim() : new ClimberIOVortex(); // TODO
    private final SuperStructure upperParts = new SuperStructure(shooterIO, intakeIO, hopperIO, climberIO);

    // SysID Routine for flywheels(shooter/hopper/intake)
    private final SysIdRoutine flywheelRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Voltage.ofRelativeUnits(3, Units.Volts), null,
            (state) -> Logger.recordOutput("flywheelRoutine", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            // Change this line to characterize other motors
            (voltage) -> shooterIO.runShooterVoltage(voltage.in(Volts)), 
            null, // No log consumer, since data is recorded by AdvantageKit
            upperParts
        )
    );

    // Commands
    private final AutoDrive autoDrive = new AutoDrive(drivetrain);
    private final Command autoIntakeCommand = upperParts.runIntake();
    private final Command autoShooterCommand = Commands.parallel(
        pathplannerDrive,
        upperParts.runShooter(() -> calculationUtil.getAimParams().pitch)
    );

    public RobotContainer() {
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        NamedCommands.registerCommand("runIntake", autoIntakeCommand);
        
        NamedCommands.registerCommand(
            "stopIntake",
            Commands.runOnce(
                () -> CommandScheduler.getInstance().cancel(autoIntakeCommand)
            )
        );

        NamedCommands.registerCommand("runShooter", autoShooterCommand);

        NamedCommands.registerCommand(
            "stopShooter",
            Commands.runOnce(
                () -> {
                    CommandScheduler.getInstance().cancel(autoShooterCommand);
                }
            )
        );

        NamedCommands.registerCommand(
            "drive27",
            new AutoDrive(drivetrain).withTarget(new Pose2d(2, 7, Rotation2d.fromDegrees(0)))
        );

        // This will use Commands.none() as the default option.
        // Displayed as "None"
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> GlobalConstants.COMP
                ? stream.filter(auto -> auto.getName().startsWith("comp_"))
                : stream
        );

        //#region SysID
        if(GlobalConstants.SYS_ID_SWERVE && !GlobalConstants.COMP){
            rotationSysID.setDefaultOption("Rotation", true);
            rotationSysID.addOption("Translation", false);
            SmartDashboard.putData("SysID Option", rotationSysID);

            autoChooser.addOption(
                "SysID | Dynamic Forward", 
                drivetrain.sysIdDynamic(Direction.kForward, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Dynamic Reverse", 
                drivetrain.sysIdDynamic(Direction.kReverse, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Quasistatic Forward", 
                drivetrain.sysIdQuasistatic(Direction.kForward, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Quasistatic Reverse", 
                drivetrain.sysIdQuasistatic(Direction.kReverse, rotationSysID.getSelected())
            );
        }

        if(GlobalConstants.SYS_ID_FLYWHEELS && !GlobalConstants.COMP){
            autoChooser.addOption("Shooter SysID | DF", flywheelRoutine.dynamic(Direction.kForward));
            autoChooser.addOption("Shooter SysID | DR", flywheelRoutine.dynamic(Direction.kReverse));
            autoChooser.addOption("Shooter SysID | QF", flywheelRoutine.quasistatic(Direction.kForward));
            autoChooser.addOption("Shooter SysID | QR", flywheelRoutine.quasistatic(Direction.kReverse));
        }

        //#endregion

        drivetrain.setDefaultCommand(teleDrive);

        SmartDashboard.putData("Auto to Run", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        //#region Bindings
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driverController.leftTrigger(0.15).debounce(0.02).whileTrue(
            upperParts.runIntake()
        );

        autoAimTrigger.whileTrue(
            upperParts.runShooter(() -> calculationUtil.getAimParams().pitch)
        );

        /*
        driverController.b().whileTrue(
            upperParts.openClimber() 
        );

        driverController.a().whileTrue(
            upperParts.closeClimber()
        ); 

        autoAim.whileTrue(
            shooterCommand
        );
        */

        // TODO drivetest, a climb position manager maybe
        driverController.a().onTrue(autoDrive.withTarget(new Pose2d(5, 5, Rotation2d.fromDegrees(0))));
        
        driverController.x().onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(2.5, 4.5, Rotation2d.kZero)) // BLUE
            )
        );
        driverController.b().onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(14, 4.5, Rotation2d.k180deg)) // RED
            )
        );
        //#endregion
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
