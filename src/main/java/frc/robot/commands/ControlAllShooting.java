package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

public class ControlAllShooting extends Command
{

  private final Supplier<Pose2d> goalPoseSupplier;
  private final Shooter m_shooter;
  private final Supplier<Pose2d> robotPoseSupplier;


  public double distance = 0.0;
  public double RecordedidealHorizontalSpeed;
  private boolean isCASAtSpeed = false;


  public boolean isCASAtSpeed() { // <-- getter for RobotContainer
    return isCASAtSpeed;
  }
  private static final InterpolatingDoubleTreeMap shooterTable = ShooterConstants.shooterTable;

  public ControlAllShooting(Supplier<Pose2d> goalPoseSupplier, Shooter shooter, Supplier<Pose2d> robotPoseSupplier)
  {
    this(goalPoseSupplier, shooter, robotPoseSupplier, false);
  }

  public ControlAllShooting(Supplier<Pose2d> goalPoseSupplier, Shooter shooter, Supplier<Pose2d> robotPoseSupplier, boolean requireShooter) {
    this.goalPoseSupplier = goalPoseSupplier;
    this.m_shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    if(requireShooter) addRequirements(m_shooter);
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    Translation2d goalLocation = goalPoseSupplier.get().getTranslation();
    Translation2d robotLocation = robotPoseSupplier.get().getTranslation();
    Translation2d targetVec = goalLocation.minus(robotLocation);
    double        dist         = targetVec.getNorm();
    distance = dist;
    double idealHorizontalSpeed = shooterTable.get(dist);

    BooleanSupplier isAtSpeed = () -> Math.abs(idealHorizontalSpeed + m_shooter.RPMOffset - m_shooter.getRPM()) <= ShooterConstants.ERROR_MARGIN;

    m_shooter.setTargetRPM(idealHorizontalSpeed + m_shooter.RPMOffset); 
    RecordedidealHorizontalSpeed = idealHorizontalSpeed;

    if (isAtSpeed.getAsBoolean()) {
    isCASAtSpeed = true; // Set the flag to true when at speed
    }
    else {
    isCASAtSpeed = false; // Set the flag to false when not at speed
    }
     
     Logger.recordOutput("Shooter/LUTCurrentTargetRPM", RecordedidealHorizontalSpeed);
     Logger.recordOutput("Shooter/LUTDistance", dist);
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