package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ControllAllPassing extends Command
{

  private final Pose2d goalPose;
  private final Shooter m_shooter;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldVelocitySupplier;

  public double RecordedidealHorizontalSpeed;
  private boolean isCASAtSpeed = false;
  private Pose2d virtualAimPose;

  public boolean isCASAtSpeed() {
    return isCASAtSpeed;
  }

  public Pose2d getVirtualAimPose() {
    return virtualAimPose != null ? virtualAimPose : goalPose;
  }

  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();


  public ControllAllPassing(Pose2d goalPose, Shooter shooter,
                             Supplier<Pose2d> robotPoseSupplier,
                             Supplier<ChassisSpeeds> fieldVelocitySupplier)
  {
    this.goalPose = goalPose;
    this.m_shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldVelocitySupplier = fieldVelocitySupplier;

    //RPM = 249.665 v_out
    for (var entry : List.of(
                          Pair.of(Meters.of(2), RPM.of(1622.8225)),
                          Pair.of(Meters.of(3), RPM.of(1900)),
                          Pair.of(Meters.of(4), RPM.of(2200)),
                          Pair.of(Meters.of(5.2048), RPM.of(2296.918))
                          )
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements();
  }

  @Override
  public void initialize()
  {
    virtualAimPose = goalPose;
  }

  @Override
  public void execute()
  {
    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds robotSpeed = fieldVelocitySupplier.get();
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

    // 1. LATENCY COMP — project robot position forward by system latency
    Translation2d futurePos = robotPose.getTranslation().plus(
        robotVelVec.times(ShooterConstants.SHOOT_LATENCY_SEC)
    );

    // 2. GET TARGET VECTOR from future position to goal
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // 3. LOOK UP STATIONARY RPM for this distance
    double stationaryRPM = shooterTable.get(dist);

    // 4. CONVERT stationary RPM to horizontal m/s
    double stationaryMPS = stationaryRPM / ShooterConstants.RPM_TO_MPS_FACTOR;

    // 5. VECTOR SUBTRACTION — compute the shot vector the shooter must produce
    Translation2d shotVec = targetVec.div(dist).times(stationaryMPS).minus(robotVelVec);

    // 6. CONVERT back to RPM
    double compensatedMPS = shotVec.getNorm();
    double compensatedRPM = compensatedMPS * ShooterConstants.RPM_TO_MPS_FACTOR;

    // 7. COMPUTE virtual aim pose from the shot vector direction
    Rotation2d shotAngle = shotVec.getAngle();
    Translation2d virtualAimPoint = robotPose.getTranslation().plus(
        new Translation2d(dist, shotAngle)
    );
    virtualAimPose = new Pose2d(virtualAimPoint, shotAngle);

    // 8. SET OUTPUTS
    m_shooter.setTargetRPM(compensatedRPM);
    RecordedidealHorizontalSpeed = compensatedRPM;

    BooleanSupplier isAtSpeed = () -> Math.abs(compensatedRPM - m_shooter.getRPM()) <= ShooterConstants.ERROR_MARGIN;
    isCASAtSpeed = isAtSpeed.getAsBoolean();

    // Logging
    Logger.recordOutput("Passer/LUTCurrentTargetRPM", compensatedRPM);
    Logger.recordOutput("Passer/VirtualAimPose", virtualAimPose);
    Logger.recordOutput("Passer/CompensatedRPM", compensatedRPM);
    Logger.recordOutput("Passer/StationaryRPM", stationaryRPM);
    Logger.recordOutput("Passer/DistToTarget", dist);
    Logger.recordOutput("Passer/RobotSpeedMPS", robotVelVec.getNorm());
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    m_shooter.stopShooting();
  }
}
