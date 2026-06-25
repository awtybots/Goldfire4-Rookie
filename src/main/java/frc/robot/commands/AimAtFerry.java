package frc.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import java.util.List;
import swervelib.SwerveInputStream;

public class AimAtFerry extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final SwerveInputStream swerveInputStream;

  public AimAtFerry(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy()
        .aim(swerveSubsystem::getCachedDynamicFerryLocation)
        .aimFeedforward(0.00045, 0.0001, 0.00022)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true);
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.setAimLocations();
    swerveSubsystem.isAiming = true;
    swerveInputStream.aimWhile(true);
  }

  @Override
  public void execute() {
    // shoot at the nearest ferry pos
    // Pose2d ferry = swerveSubsystem.getCachedDynamicFerryLocation();

    swerveSubsystem.driveFieldOriented(swerveInputStream.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.isAiming = false;
    swerveInputStream.aimWhile(false);
    // swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}