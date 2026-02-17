// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveTelemetry;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.utils.AutoAim;
import frc.robot.utils.AxisConfigLoader;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();

    // Swerve
    private final Drive drivetrain = TunerConstants.createDrivetrain();
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();     
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Trigger autoAim = driverController.rightBumper().debounce(0.02);
    private final AutoAim calculationUtil = new AutoAim(drivetrain);

    private final DriveCommand teleDrive = new DriveCommand(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> autoAim.getAsBoolean(),
        AxisConfigLoader.loadTable(GlobalConstants.LEFT_AXIS_CONFIG),
        AxisConfigLoader.loadTable(GlobalConstants.RIGHT_AXIS_CONFIG),
        () -> calculationUtil.getAimParams()
    );

    // SuperStructure    
    private final ShooterIO shooterIO = Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOSim();
    private final IntakeIO intakeIO = Utils.isSimulation() ? new IntakeIOSim() : new IntakeIOSim();
    private final HopperIO hopperIO = Utils.isSimulation() ? new HopperIOSim() : new HopperIOSim();
    private final ClimberIO climberIO = Utils.isSimulation() ? new ClimberIOSim() : new ClimberIOSim();
    private final SuperStructure upperParts = new SuperStructure(shooterIO, intakeIO, hopperIO, climberIO);

    // Auto Aim Calculation
    

    public RobotContainer() {
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        // This will use Commands.none() as the default option.
        // Displayed as "None"
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> GlobalConstants.COMP
                ? stream.filter(auto -> auto.getName().startsWith("comp_"))
                : stream
        );

        //#region SysID
        if(GlobalConstants.SYS_ID){
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

        driverController.leftTrigger(0.15).whileTrue(
            upperParts.runIntake()
        );

        driverController.b().onTrue(
            upperParts.runClimber() 
        );

        autoAim.whileTrue(
            upperParts.runShooter(
                () -> calculationUtil.getAimParams().pitch
            )
        );

        // driverController.a().onTrue(new AutoDrive(drivetrain).withTarget(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        //#endregion
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
