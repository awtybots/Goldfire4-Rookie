package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ControlAllShooting;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubTrackerSubsystem extends SubsystemBase
{

    private Field2d field = new Field2d();
    private final SwerveSubsystem drivebase;
    private FieldObject2d circle;
    private FieldObject2d traj;

    final CommandXboxController driverController;

    private Pose2d hubPose;
    private double radius = 0.5; // radius to represent time left (circle gets smaller when shift ending)

    boolean show = false;
    
    int x = 0;
    
    public HubTrackerSubsystem(SwerveSubsystem drivebase, CommandXboxController driverController)
    {
        this.drivebase = drivebase;
        circle = field.getObject("Circle"); 
        traj = field.getObject("Trajectory"); 
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        hubPose = switch (alliance)
        {
            case Blue -> new Pose2d(4.6, 4, new Rotation2d());
            case Red -> new Pose2d(11.9, 4, new Rotation2d());
            default -> new Pose2d(4.6, 4.1, new Rotation2d());
        };
        SmartDashboard.putData("Field", field);
        this.driverController = driverController;
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty())
        {
          SmartDashboard.putNumber("TimeLeft", -1);
          return false;
        }
        if(!DriverStation.isFMSAttached())
        {
          SmartDashboard.putNumber("TimeLeft", -1);
          return false;
        }

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomousEnabled()) // Always enabled in auton
        {
            int seconds = (20 - (int)matchTime);
            SmartDashboard.putNumber("TimeLeft", seconds);
            vibrate(seconds / 20.0);
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {SmartDashboard.putNumber("TimeLeft", -1);return false;} // If we're disabled then were probably not playing

        
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;

        boolean isRedActiveFirst;
        switch (gameData.charAt(0)) {
        case 'R':
            isRedActiveFirst = false;
            break;
        case 'B':
            isRedActiveFirst = true;
            break;
        default:
            SmartDashboard.putNumber("TimeLeft", -1);
            return false;
        }

        boolean shiftOneActive = switch (alliance.get()) {
        case Red -> isRedActiveFirst;
        case Blue -> !isRedActiveFirst;
        };

        if (matchTime >= 130) // transition shift, always active
        {
            publish(140, (int)matchTime, 10.0);
            return true;
        } 
        else if (matchTime >= 105) { // shift 1
            publish(130, (int)matchTime, 25.0);
            return shiftOneActive;
        } 
        else if (matchTime >= 80) // shift 2
        {
            publish(105, (int)matchTime, 25.0);
            return !shiftOneActive;
        } 
        else if (matchTime >= 55) // shift 3
        {
            publish(80, (int)matchTime, 25.0);
            return shiftOneActive;
        } 
        else if (matchTime >= 30) // shift 4
        {
            publish(55, (int)matchTime, 25.0);
            return !shiftOneActive;
        } 
        else
        {
            int seconds = (int)matchTime;
            SmartDashboard.putNumber("TimeLeft", seconds);
            vibrate(seconds / 30.0);
            return true; // Endgame, always active
        }
  }

  private void publish(int time, int matchTime, double shiftTime)
  {
    int seconds = (time - matchTime);
    SmartDashboard.putNumber("TimeLeft", seconds);
    radius = seconds / shiftTime;
    vibrate(radius);
  }

//   public Command isHubActiveCommand()
//   {
//     return run(()->
//     {
//         isHubActive();
//     });
//   }

  private void vibrate(double r)
  {
    if(r <= 0.15 && DriverStation.isFMSAttached())
    {
        driverController.setRumble(RumbleType.kBothRumble, 0.15 * Math.pow((1.0 - r), 2));
    }
    else driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  public List<Pose2d> createCircle(Pose2d center, double r, int pts)
  {
    List<Pose2d> circle = new ArrayList<>();
    for(int i=0;i<pts;++i)
    {
        double angle = (i * (360.0/(double)pts)) * Math.PI / 180;
        
        double xOffset = r * Math.cos(angle);
        double yOffset = r * Math.sin(angle);

        circle.add(new Pose2d(center.getX() + xOffset, center.getY() + yOffset, new Rotation2d()));
    }


    return circle;
  }

  public void runPeriodic()
  {
    Pose2d robotPose = drivebase.getPose();

    field.setRobotPose(robotPose);
    boolean active = isHubActive();
    x = (x == 1) ? 0 : 1;
    // Ok so this is a really weird (but genius) line of code i came up with (yk it aint AI because AI wouldn't make something like this lol)
    // essentially, the circle will be shown either when it is an INACTIVE hub, or every other frame of active shift
    // this way, if it is active, it blinks. this is my workaround for not being able to change color of circle
    SmartDashboard.putBoolean("HubActivity", active);
    if(!active || x == 0) circle.setPoses(createCircle(drivebase.getDynamicHubLocation(), radius, 20));
    else circle.setPoses(); // clears circle when not showing

    Translation2d goalLocation = drivebase.getDynamicHubLocation().getTranslation();
    Translation2d robotLocation = robotPose.getTranslation();
    Translation2d targetVec = goalLocation.minus(robotLocation);
    double        dist         = targetVec.getNorm();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(robotPose, List.of(), robotPose.plus(new Transform2d(-2, 0, new Rotation2d())), new TrajectoryConfig(1.0, 1.0));
    traj.setTrajectory(trajectory);

    SmartDashboard.putNumber("Distance to Hub", dist);
    // SmartDashboard.putNumber("LUTRPM", ControlAllShooting.getRPM(dist));
  }

  @Override
  public void periodic()
  {
    runPeriodic();
  }

  @Override
  public void simulationPeriodic()
  {
    runPeriodic();
  }
}