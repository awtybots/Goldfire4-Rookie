package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class AimAtTarget extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final SwerveInputStream swerveInputStream;

  public boolean readyToLock = false;

  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;

  public AimAtTarget(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream,
      DoubleSupplier leftX, DoubleSupplier leftY) {
    this.swerveSubsystem = swerveSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.swerveInputStream = swerveInputStream.copy()
        .aim(swerveSubsystem::getCachedDynamicAimLocation)
        .aimFeedforward(0.00045, 0.0001, 0.00022)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds));
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
    swerveSubsystem.driveFieldOriented(swerveInputStream.get());

    double leftMag = Math.hypot(leftX.getAsDouble(), leftY.getAsDouble());
    if (leftMag < Constants.OperatorConstants.DEADBAND && readyToLock) {
      swerveSubsystem.lock();
      SmartDashboard.putBoolean("Wheel Lock", true);
    } else {
      SmartDashboard.putBoolean("Wheel Lock", false);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.isAiming = false;
    swerveInputStream.aimWhile(false);
  }
}
