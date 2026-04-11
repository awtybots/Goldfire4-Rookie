package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class AimAtHub extends Command {

    public final SwerveSubsystem swerveSubsystem;
    public final SwerveInputStream swerveInputStream;

    public boolean readyToLock = false;

    private final DoubleSupplier leftX;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;

    public AimAtHub(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream,
            DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.swerveSubsystem = swerveSubsystem;
        this.swerveInputStream = swerveInputStream.copy();
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
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
        swerveInputStream
                .aim(swerveSubsystem::getCachedDynamicHubLocation) // supplier, updates each loop
                .aimFeedforward(0.00045, 0.0001, 0.00022);
        double leftMag = Math.hypot(leftX.getAsDouble(), leftY.getAsDouble());
        double rightMag = Math.abs(rightX.getAsDouble());
        swerveSubsystem.driveFieldOriented(swerveInputStream.get());


        if ((leftMag + rightMag) > (Constants.OperatorConstants.DEADBAND + 0.2) && !readyToLock) {
            SmartDashboard.putBoolean("Wheel Lock", false);
        } else {
            // swerveSubsystem.lock();
            SmartDashboard.putBoolean("Wheel Lock", true);
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
        swerveSubsystem.stop();
    }
}