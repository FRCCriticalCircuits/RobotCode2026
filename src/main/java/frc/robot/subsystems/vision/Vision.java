package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase{
    private final VisionIO[] listVisionIO;
    private final Drive drivetrain;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(Drive drivetrain, VisionIO... visionIOs){
        this.drivetrain = drivetrain;
        this.listVisionIO = visionIOs;
    }

    @Override
    public void periodic() {
        for (VisionIO visionIO : listVisionIO) {
            visionIO.updateInputs(inputs);
            Logger.processInputs("VisionResult/" + visionIO.toString(), inputs);

            for (var observedData : inputs.poseObservation) {
                drivetrain.addVisionMeasurement(
                    observedData.robotPose(),
                    observedData.timeStamp()
                );
            }
        }
    }
}
