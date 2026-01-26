// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveTelemetry;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();

    //#region Swerve
    private final Drive drivetrain = TunerConstants.createDrivetrain();
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();     
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final AutoDrive autoDriveCommand = new AutoDrive(drivetrain);
    private final DriveCommand teleDrive = new DriveCommand(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.button(0)
            .debounce(0.04)
            .getAsBoolean()
    );

    //#endregion

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

        //#region Subsystem Commands
        drivetrain.setDefaultCommand(teleDrive);

        SmartDashboard.putData("Auto to Run", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        //#region Bindings - drive
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().debounce(0.04).whileTrue(
            autoDriveCommand.withTarget(
                new Pose2d(5, 5, Rotation2d.fromDegrees(0))
            )
        );

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //#endregion
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
