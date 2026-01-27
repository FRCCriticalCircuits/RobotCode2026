package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.CAD;

public class ShooterTest extends SubsystemBase {
    public final ShooterIO io = new ShooterIOSim();
    public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private double pitch, velocity;

    public ShooterTest() {}

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        io.runHood(pitch);
        io.runShooter(velocity);

        visualization();
    }

    public void setPitch(double position){
        this.pitch = position;
    }

    public void runVelocity(double velocity){
        this.velocity = velocity;
    }

    public void visualization(){
        Logger.recordOutput("Visualization/Hood", 
            new Pose3d(
                CAD.ORIGIN_OFFSET_X,
                CAD.ORIGIN_OFFSET_Y, 
                CAD.ORIGIN_OFFSET_Z,
                new Rotation3d(
                    0,
                    inputs.hoodPosition + CAD.PITCH_OFFSET,
                    0
                )
            )
        );

        
        Logger.recordOutput("Visualization/IntakeTest", 
            new Pose3d(
                0.324, 
                -0.349, 
                0.194,
                new Rotation3d(
                    0,
                    -inputs.hoodPosition,
                    0
                )
            )
        );
    }
}
