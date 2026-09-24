package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private double aimTolerance(double distance) {
    if (distance < 2)
      return 5.0;
    else if (distance < 3.5)
      return 2.0;
    return 1.0;
  }

  @Override
  public void initialize() {
    swerveSubsystem.setAimLocations();
    swerveSubsystem.isAiming = true;
    swerveInputStream.aimWhile(true);
    readyToLock = false;
  }

  @Override
  public void execute() {
    swerveSubsystem.driveFieldOriented(swerveInputStream.get());

    // SwerveInputStream.aimLock() compares angles without wrapping, so it never fires when the
    // goal sits near +-180 deg - which is every hub shot, since the shooter points out the back.
    Translation2d toTarget = swerveSubsystem.getCachedDynamicAimLocation().getTranslation()
        .minus(swerveSubsystem.getPose().getTranslation());
    double distance = toTarget.getNorm();
    double aimErrorDegrees = Math.abs(MathUtil.inputModulus(
        toTarget.getAngle().minus(swerveSubsystem.getPose().getRotation()).getDegrees() - 180.0,
        -180.0, 180.0));
    readyToLock = aimErrorDegrees <= aimTolerance(distance);

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
    readyToLock = false;
    swerveSubsystem.isAiming = false;
    swerveInputStream.aimWhile(false);
  }
}
