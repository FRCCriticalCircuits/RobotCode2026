package frc.robot.subsystems.intake;

import frc.robot.enums.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    public static Intake instance = null;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private IntakeState curState = IntakeState.STOP;

    private Intake(IntakeIO io){
        this.io = io;
    }

    public static Intake getInstance(IntakeIO io){
        if(instance == null) instance = new Intake(io);
        return instance;
    }

    public void setState(IntakeState state){
        this.curState = state;
    }

    @Override
    public void periodic() {
        switch (curState) {
            case INTAKING:
                io.setSlider(IntakeConstants.EXTENDED);
                io.runRoller(IntakeConstants.VOLTS_INTAKE);
                break;
            case REVERSE:
                io.setSlider(IntakeConstants.EXTENDED);
                io.runRoller(IntakeConstants.VOLTS_OUTTAKE);
                break;
            default:
                io.setSlider(IntakeConstants.RETRACT);
                io.runRoller(0);
                break;
        }

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Visualization/Slider", visualize(inputs.positionSlider));
        // will delete later
        Logger.recordOutput("Visualization/Hood", new Pose3d(new Translation3d(), new Rotation3d()));
    }

    private Pose3d visualize(double sliderPosition){
        // minor GC?
        return new Pose3d(
            new Translation3d(
                -sliderPosition * IntakeConstants.COS_SLIDER_DIR,
                0,
                -sliderPosition * IntakeConstants.SIN_SLDIER_DIR
            ),
            new Rotation3d(
                0,
                0,
                0
            )
        );
    }
}
