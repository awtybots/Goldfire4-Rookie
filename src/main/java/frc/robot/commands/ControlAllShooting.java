package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Hopper;
// import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Pushout;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import swervelib.SwerveDrive;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;


/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ControlAllShooting extends Command
{

  private final Supplier<Pose2d> goalPoseSupplier;
  private final Shooter m_shooter;
  private final Supplier<Pose2d> robotPoseSupplier;
  // private final Hopper m_hopper;
  // private final Kicker m_kicker;
  // private final Pushout m_pushout;

  public double RecordedidealHorizontalSpeed;
    private boolean isCASAtSpeed = false;

  // WaitCommand wait = new WaitCommand(1);

    public boolean isCASAtSpeed() { // <-- getter for RobotContainer
    return isCASAtSpeed;
  }
  // private final SwerveSubsystem m_swerveSubsystem;
  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  // private final double                     latency      = 0.15;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
  
  

  public ControlAllShooting(Supplier<Pose2d> goalPoseSupplier, Shooter shooter, Supplier<Pose2d> robotPoseSupplier)

  {

    this.goalPoseSupplier = goalPoseSupplier;
    this.m_shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    // this.m_swerveSubsystem = swerveSubsystem;
    // this.m_hopper = hopper;
    // this.m_kicker = kicker;
    // 4.034 meters half field, 0.661 byumper to shooter exit. Only 3.373 vertical distance to target meters, 
    // horizontal distance 4.625 meters from driver station to middle of hub, minus 0.661 byumper to shooter exit, 
    // total 3.964 meters horizontal distance from driver station to shooter exit.
    //using cosine rule we find that max distance would be 5.2048 M
    //the closest shooter can get to the hub would be 1.1277 meters
    //ball exit at 62 degrees
    //ball exit from 0.391 meters above ground.
    //RPM = 249.665 v_out
    // Test Results
    
    for (var entry : List.of(
      // Pair.of(Meters.of(1), RPM.of((1000))),
                            Pair.of(Meters.of(2), RPM.of(1700)),
                            Pair.of(Meters.of(2.5), RPM.of(1815)),
                            Pair.of(Meters.of(3), RPM.of(1930)),
                            Pair.of(Meters.of(3.5), RPM.of(2090)),
                            // Pair.of(Meters.of(3.5), RPM.of(2050),
                            Pair.of(Meters.of(4), RPM.of(2260)),
                            Pair.of(Meters.of(5.2048), RPM.of(2456.918)),
                            Pair.of(Meters.of(6), RPM.of(2600))
                            
                            //  Pair.of(Meters.of(2), RPM.of(1700)),
                            // Pair.of(Meters.of(2.5), RPM.of(1815)),
                            // Pair.of(Meters.of(3), RPM.of(1930)),
                            // Pair.of(Meters.of(3.5), RPM.of(2045)),
                            // // Pair.of(Meters.of(3.5), RPM.of(2050),
                            // Pair.of(Meters.of(4), RPM.of(2160)),
                            // Pair.of(Meters.of(4.5), RPM.of(2275)),
                            // Pair.of(Meters.of(5), RPM.of(2390)),
                            //  Pair.of(Meters.of(5.5), RPM.of(2505)),
                            // // Pair.of(Meters.of(5.2048), RPM.of(2456.918)),
                            // Pair.of(Meters.of(6), RPM.of(2620))
                            )
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements();
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    // var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    // Translation2d futurePos = robotPose.get().getTranslation().plus(
    //     new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
    //                                                                );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPoseSupplier.get().getTranslation();
    Translation2d robotLocation = robotPoseSupplier.get().getTranslation();
    Translation2d targetVec = goalLocation.minus(robotLocation);
    double        dist         = targetVec.getNorm();
    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // Local predicate to check if shooter is at the desired speed (methods cannot be declared inside methods)
    BooleanSupplier isAtSpeed = () -> Math.abs(idealHorizontalSpeed - m_shooter.getRPM()) <= ShooterConstants.ERROR_MARGIN;
    // 4. VECTOR SUBTRACTION
    // Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    // Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    
    // double newHorizontalSpeed = shotVec.getNorm();

    // // 6. SOLVE FOR NEW PITCH/RPM
    // // Assuming constant total exit velocity, variable hood:
    // double totalExitVelocity = 15.0; // m/s
    // // Clamp to avoid domain errors if we need more speed than possible
    // double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    // double newPitch = Math.acos(ratio);

    // 7. SET OUTPUTS
     
    // Could also just set the swerveDrive to point towards this angle like AlignToGoal
    //hood.setAngle(Math.toDegrees(newPitch));
    //shooter.setRPM(MetersPerSecond.of(totalExitVelocity));
    
    


  // Commands.parallel(
  //       // keep running the VariableShoot command while we wait for the shooter to reach speed
  //       m_shooter.setTargetRPMCommand(idealHorizontalSpeed),
        
  //       // once at speed, run hopper + kicker
  //       Commands.sequence(
  //         Commands.waitUntil(isAtSpeed),
  //         Commands.parallel(
  //            m_hopper.runHopperToShooterCommand(),
  //            m_kicker.kickCommand()
  //         )
  //       )
  //    );

     m_shooter.setTargetRPM(idealHorizontalSpeed); 
     RecordedidealHorizontalSpeed = idealHorizontalSpeed;

     if (isAtSpeed.getAsBoolean()) {
      isCASAtSpeed = true; // Set the flag to true when at speed
     }
     else {
      isCASAtSpeed = false; // Set the flag to false when not at speed
     }
    //  if (isAtSpeed.getAsBoolean()) {
    //   m_hopper.HopperToShooter();
    //   m_kicker.Kick();
    //   // m_pushout.Agitate();
    //  }

     
     Logger.recordOutput("Shooter/LUTCurrentTargetRPM", RecordedidealHorizontalSpeed);
     Logger.recordOutput("Shooter/LUTDistance", dist);
    
    // m_hopper.runReverseHopperCommand().onlyIf(m_shooter::isShooterFast);
    // m_kicker.kickBackwardsCommand().onlyIf(m_shooter::isShooterFast);
    


  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    // Translation2d goalLocation = goalPose.getTranslation();
    // Translation2d robotLocation = robotPose.getTranslation();
    // Translation2d targetVec = goalLocation.minus(robotLocation);
    // double        dist         = targetVec.getNorm();

    // double idealHorizontalSpeed = shooterTable.get(dist);


    // m_shooter.setTargetRPM(idealHorizontalSpeed);
    // try {
    //   Thread.sleep(2000);
    // } catch (InterruptedException e) {
    //   Thread.currentThread().interrupt();
    //   Logger.recordOutput("ThreadSleepFrom2SecondsExtra", e.getMessage());

    // }
    m_shooter.stopShooting();
        
  }
}