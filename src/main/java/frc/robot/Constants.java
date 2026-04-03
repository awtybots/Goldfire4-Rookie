package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.math.Matter;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean USE_ROBOT_RELATIVE = false;
  public static final boolean USE_DRIVE_ONLY = false;
  public static final boolean USE_SHOOTER_ONLY = false;
  public static final boolean SIM_REPLAY_MODE = false;
  // not used
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  // used
  public static final double MAX_SPEED = Units.feetToMeters(16.5);

  // RobotContainer or a constants class
public static final double LOOKAHEAD_BASE_SEC = 0.03;    // minimum lead
public static final double LOOKAHEAD_K_OMEGA   = 0.4; //0.012 // seconds per (rad/s)
public static final double LOOKAHEAD_K_V       = 0.015;  // seconds per (m/s)
public static final double LOOKAHEAD_MIN_SEC   = 0.0;
public static final double LOOKAHEAD_MAX_SEC   = 1.5;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    public static final Pose3d redHubPose = new Pose3d(Units.inchesToMeters(469.09488), Units.inchesToMeters(158.6614),
        Units.inchesToMeters(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Units.inchesToMeters(182.12598), Units.inchesToMeters(158.6614),
        Units.inchesToMeters(72.0), new Rotation3d());

    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    // public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

    public static final Pose2d getHubPose2D() {
      Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      Pose2d Tdpose = pose.toPose2d();
      return Tdpose;
    }    
    // should i add <Supplier> to the method signature? it compiles without it but
    // im not sure if its correct
    public static final <Supplier> Pose3d getHubPose3D() {
      Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
      return pose;
    }

    public static final Pose2d getFerryPose(Translation2d robotPose) {
      if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return redFerryPoseOutpost.toPose2d();
        } else {
          return redFerryPoseDepot.toPose2d();
        }
      } else {
        if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return blueFerryPoseOutpost.toPose2d();
        } else {
          return blueFerryPoseDepot.toPose2d();
        }
      }
    }

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class LimelightConstants
  {
    public static final String LIMELIGHT_FRONT = "limelight-front";
    public static final String LIMELIGHT_BACK = "limelight-back";
    public static final String LIMELIGHT_LEFT = "limelight-left";
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int INTAKE_LEFT_ID = 18; // unknown
    public static final int INTAKE_RIGHT_ID = 19; // unknown

    // PID Constants
    public static final double p = 0.006155;
    public static final double i = 0.000;
    public static final double d = 0.01;

    // Feed-Forward Constants
    public static final double s = 1.25;
    public static final double v = 0.5;
    public static final double a = 0.75;

    public static final double OUTTAKE_SPEED = -1;
    public static final double INTAKE_SPEED = 1;
     public static final double INTAKE_RPM = -10000;
    public static final double OUTTAKE_RPM = 10000;

  }

  public static class PushoutConstants {
    public static final int PUSHOUT_ID = 17; // Correct

    public static final double PUSHOUT_RETRACTED_POS = 2; // 
    public static final double PUSHOUT_EXTENDED_POS = 14; // TUNE THIS!!!

    public static final double PUSHOUT_RETRACTED_AGITATE_POS = 2; // it was 4.693
    public static final double PUSHOUT_EXTENDED_AGITATE_POS = 11; // TUNE THIS!!!

    public static final double PUSHOUT_AGITATE_WAIT = 0.2;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_L1_ID = 9;
    public static final int SHOOTER_L2_ID = 10;
    
    public static final int SHOOTER_R1_ID = 11;
    public static final int SHOOTER_R2_ID = 12;

    public static final double SHOOTER_SPEED = -1735; // RPM 3 meters 1900 4 meters 2200
    public static final double ERROR_MARGIN = 100; // RPM
    public static final double STOP = 0;
    public static final double IDLE = 0.1; // % voltage -1 --> 1

    public static final double ALLIANCE_IDLE_RPM = -1000;
    public static final double NEUTRAL_IDLE_RPM = -500;

    // PID Constants For Shooter
    public static final double p = 0.0002355;
    public static final double i = 0.000;
    public static final double d = 0;

    // Feed-Forward Constants for Shooter
    public static final double s = 0.0;
    public static final double v = 0.0019;
    public static final double a = 0.0;

    public final static InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();

    static {
      for (var entry : List.of(
          Pair.of(Meters.of(2), Seconds.of(0.85)),
          Pair.of(Meters.of(3), Seconds.of(0.9)),
          Pair.of(Meters.of(4), Seconds.of(1.08)),
          Pair.of(Meters.of(5), Seconds.of(1.2)),
          Pair.of(Meters.of(6), Seconds.of(1.4)))) {
        TOF.put(entry.getFirst().in(Meters), entry.getSecond().in(Seconds));
      }
    }
  }

  public static class KickerConstants {
    public static final int KICKER_LEFT_ID = 13;
    public static final int KICKER_RIGHT_ID = 14;

    public static final double KICKER_REVERSE_RPM_TARGET = -10000; // RPM
    public static final double KICKER_RPM_TARGET = 10000; // RPM

    // PID Constants
    public static final double p = 0.0002355;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.003;
    public static final double a = 0.0003;

    public static final double STOP = 0;
    public static final double IDLE = 0; // % voltage -1 --> 1
  }

  public static class HopperConstants {
    // IDEAL mapping from motor_can_ids.csv: left=18, right=19
    public static final int TWINDEXER_LEFT_ID = 15;
    public static final int TWINDEXER_RIGHT_ID = 16;

    public static final double TWINDEXER_RIGHT_RPM = -0.8;
    public static final double TWINDEXER_LEFT_RPM = 0.8;

    public static final double REVERSE_TWINDEXER_RIGHT_RPM = -9000;
    public static final double REVERSE_TWINDEXER_LEFT_RPM = 9000;

    // PID Constants
    public static final double p = 0.0002;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.00177;
    public static final double a = 0.00017;

    public static final int six_seven = 67; // <---------- HISTORICAL MONUMENT

  }
  
  public static class FunnelConstants {
    // IDEAL mapping from motor_can_ids.csv: left=18, right=19
    public static final int FUNNEL_ID = 15;

    public static final double FUNNEL_RPM = -9000;
    public static final double REVERSE_FUNNEL_RPM = 9000;

    // PID Constants
    public static final double p = 0.0002;
    public static final double i = 0.000;
    public static final double d = 0.000;

    // Feed-Forward Constants
    public static final double s = 0.100;
    public static final double v = 0.00177;
    public static final double a = 0.00017;

  }


  public static class ClimberConstants {
    public static final int CLIMBER_LEFT_ID = 19; // placeholder

    public static final double CLIMBER_RETRACTED_POS = 0.0; // TUNE THIS!!!
    public static final double CLIMBER_EXTENDED_POS = 0.0; // TUNE THIS!!!
  }

  public static final double X_REEF_ALIGNMENT_P = 2.1; // Proportional gain for X-axis reef alignment
  public static final double Y_REEF_ALIGNMENT_P = 2.5; // Proportional gain for Y-axis reef alignment (previously 1.74)
  public static final double ROT_REEF_ALIGNMENT_P = 0.07; // Proportional gain for rotational reef alignment
  public static final boolean USE_AUTO_ALIGNMENT_FAST_APPROACH = false; // Turn fast appraoch for auto align
  public static final double AUTO_ALIGNMENT_FAST_APPROACH_DISTANCE_METERS = 0.60; // Distance where we switch from max
                                                                                  // speed to PID control
  public static final double AUTO_ALIGNMENT_FAST_APPROACH_SPEED = 1.2; // Fast approach speed in m/s when far from the
                                                                       // reef
  // Shift these setpoints when the robot stops short, crashes the reef, or parks
  // off-center.
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Desired robot heading when aligned to the reef
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1; // Allowable heading error while aligning
  public static final double X_SETPOINT_REEF_ALIGNMENT = 0.06; // Desired X offset from reef for scoring (previously
                                                               // -0.43)
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.08; // Acceptable X error when aligning
  public static final double Y_L_SETPOINT_REEF_ALIGNMENT = 0.05; // Desired Y offset when approaching left reef side
                                                                 // (was -0.359)
  public static final double Y_R_SETPOINT_REEF_ALIGNMENT = 0.275; // Desired Y offset when approaching right reef side
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1; // Acceptable Y error during alignment

  // Extend this wait if brief vision dropouts abort alignment, shorten to bail
  // sooner.
  public static final double DONT_SEE_TAG_WAIT_TIME = 0.4; // Time to continue aligning after vision tag loss
  public static final double POSE_VALIDATION_TIME = 0.07; // Duration a pose measurement must remain valid
  public static final double POSE_LOSS_GRACE_PERIOD = 0.2; // Allowed vision dropout time before aborting alignment

  // Dashboard throttling
  public static final boolean LIMIT_DASHBOARD_PERIODIC_UPDATES = false; // Enable throttling of dashboard updates
  public static final int DASHBOARD_UPDATE_PERIOD_CYCLES = 10; // Number of periodic loops between dashboard refreshes

  // Object Detection
  public static final double X_FUEL_SETPOINT = 0.5;
  public static final double Y_FUEL_SETPOINT = 0.0;

  public static final double X_FUEL_TOLERANCE = 0.1;
  public static final double Y_FUEL_TOLERANCE = 0.1;


  public static class Dimensions {
        public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
        public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
        public static final Distance FRAME_SIZE_Y = Inches.of(26.25); // left to right (y-axis)
        public static final Distance FRAME_SIZE_X = Inches.of(28.75); // front to back (x-axis)

        public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
        public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    }
}