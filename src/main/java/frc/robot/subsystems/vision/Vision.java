package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase{
    private final VisionIO[] listVisionIO;
    private final Drive drivetrain;

    private final VisionIOInputs inputs = new VisionIOInputs();

    public Vision(Drive drivetrain, VisionIO... visionIOs){
        this.drivetrain = drivetrain;
        this.listVisionIO = visionIOs;
    }

    @Override
    public void periodic() {
        for (VisionIO visionIO : listVisionIO) {
            visionIO.updateInputs(inputs);

            for (var observedData : inputs.poseObservation) {
                drivetrain.addVisionMeasurement(
                    observedData.robotPose(),
                    observedData.timeStamp()
                );
            }
        }
    }
}
