package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
// import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ObjectDetection extends SubsystemBase {
    
    // Name of our camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // Query the latest result from PhotonVision
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // Check if the latest result has any targets.
    boolean hasTargets = (results.size() > 0);
    private PIDController xController, yController; // rotController;

    private SwerveSubsystem drivebase;


    public ObjectDetection(SwerveSubsystem drivebase) {
        // Forward/back: bump P up if the robot creeps in too slowly, drop it if it
        // shoots past the tag.
        xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);
        // Strafe: raise this P when the chassis stays offset left/right of the reef,
        // lower if it oscillates.
        yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);

        xController.setSetpoint(Constants.X_FUEL_SETPOINT); // Forward distance offset
        yController.setSetpoint(Constants.Y_FUEL_SETPOINT); // Lateral distance offset

        xController.setTolerance(Constants.X_FUEL_TOLERANCE);
        yController.setTolerance(Constants.Y_FUEL_TOLERANCE);
        // Yaw: tune this when the robot finishes facing left/right instead of square to
        // the reef.

        // rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);

        // Rotation
        // rotControllerProfiled = new
        // ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
        // new TrapezoidProfile.Constraints(6.28, 3.14));
        // Rotation using holonic drive controller

        // var controller = new HolonomicDriveController(
        // new PIDController(Constants.X_REEF_ALIGNMENT_P, 0, 0), new
        // PIDController(Constants.Y_REEF_ALIGNMENT_P, 0, 0),
        // new ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
        // new TrapezoidProfile.Constraints(6.28, 3.14)));
        // // Here, our rotation profile constraints were a max velocity
        // // of 1 rotation per second and a max acceleration of 180 degrees
        // // per second squared.
        // this.isRightScore = isRightScore;
        this.drivebase = drivebase;
  }

    public double getAlignToFuelRotation() {
        // Simple P-controller to compute the rotation command to align the robot to the target.
        // Tweak kP and maxRot for your robot.
        final double kP = 0.02;    // proportional gain (tune)
        final double maxRot = 30; // max rotational speed (tune)

        if (hasTargets) {
            var bestTarget = results.get(0).getBestTarget();
            // PhotonTrackedTarget#getYaw() returns the horizontal offset (degrees) from camera to target.
            double yawDegrees = bestTarget.getYaw();
            double rotationCmd = yawDegrees * kP;
            // clamp
            rotationCmd = Math.max(-maxRot, Math.min(maxRot, rotationCmd));
            return rotationCmd;
        } else {
            // No target: no rotation
            return 0.0;
        }
    }

    public Command Align() 
    {
        if(hasTargets)
        {
            return Commands.run(() -> {
                SmartDashboard.putBoolean("ReadyToAlign", true);

                var result = results.get(0).getBestTarget();
                var translation = result.getBestCameraToTarget().getTranslation();
                // double yaw = result.getYaw();
                double rotCmd = getAlignToFuelRotation();

                double forward = translation.getZ();
                double lateral = translation.getX();

                double xSpeed = -xController.calculate(forward);
                double ySpeed = yController.calculate(lateral);

                // boolean aligned = xController.atSetpoint() && yController.atSetpoint();

                // if we aren't aligned, drive
                drivebase.drive(new Translation2d(ySpeed, xSpeed), Math.toRadians(rotCmd), false);
                // else drivebase.stop(); // if aligned stop moving
            }, this, drivebase)
            .until(() -> (xController.atSetpoint() && yController.atSetpoint()))
            .finallyDo(() -> drivebase.stop());
        }
        else
        {
            return Commands.run(() ->
            {
                SmartDashboard.putBoolean("ReadyToAlign", false);
                drivebase.stop();
            }, this, drivebase);
        }
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();
        hasTargets = (results.size() > 0);
    }

    @Override
    public void simulationPeriodic() {
        results = camera.getAllUnreadResults();
        hasTargets = (results.size() > 0);

        Logger.recordOutput("ObjectDetection/PipelineHasTargets", hasTargets);
        
        results.forEach(result -> {
            Logger.recordOutput("ObjectDetection/TargetPose", result.getBestTarget().getBestCameraToTarget());
        });

        if (hasTargets) {
            var bestTarget = results.get(0).getBestTarget();
            var targetPose = bestTarget.getBestCameraToTarget();
            Logger.recordOutput("ObjectDetection/AlignToFuelPose", targetPose);
        }


    }

}
