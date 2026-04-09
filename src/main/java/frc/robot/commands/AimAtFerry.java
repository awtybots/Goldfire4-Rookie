package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import swervelib.SwerveInputStream;

public class AimAtFerry extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final SwerveInputStream swerveInputStream;

  public AimAtFerry(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.isAiming = true;
    swerveInputStream
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true)
        .aimWhile(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds));

  }

  @Override
  public void execute() {
    // shoot at the nearest ferry pos
    swerveInputStream
            .aim(swerveSubsystem::getCachedDynamicFerryLocation) // supplier, updates each loop
            .aimFeedforward(0.00045, 0.0001, 0.00022);
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
    swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}