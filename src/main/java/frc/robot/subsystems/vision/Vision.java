package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase{
    private final VisionIO[] listVisionIO;
    private final Drive drivetrain;
    private final SwerveDriveState driveState;

    private final VisionIOInputs inputs = new VisionIOInputs();

    private boolean m_visionDisabled = true;

    public Vision(Drive drivetrain, VisionIO... visionIOs){
        this.drivetrain = drivetrain;
        this.listVisionIO = visionIOs;

        this.driveState = drivetrain.getState();
    }

    public void setVisionEnabled() {
        m_visionDisabled = false;
    }

    public void setVisionDisabled() {
        m_visionDisabled = true;
    }

    @Override
    public void periodic() {
        if(m_visionDisabled) return;

        for (VisionIO visionIO : listVisionIO) {
            visionIO.feedPose(driveState.Pose);
            visionIO.updateInputs(inputs);

            for (var observedData : inputs.poseObservation) {
                if(observedData == null) continue;

                drivetrain.addVisionMeasurement(
                    observedData.robotPose(),
                    observedData.timeStamp(),
                    observedData.stdDev()
                );
            }
        }
    }
}
