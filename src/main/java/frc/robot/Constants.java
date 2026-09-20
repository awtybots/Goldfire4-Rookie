package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final double ROBOT_MASS = 130 * 0.453592;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  // used
  public static final double MAX_SPEED = Units.feetToMeters(17.5);

  // RobotContainer or a constants class
  public static final double LOOKAHEAD_BASE_SEC = 0.03; // minimum lead
  public static final double LOOKAHEAD_K_OMEGA = 0.4; // 0.012 // seconds per (rad/s)
  public static final double LOOKAHEAD_K_V = 0.015; // seconds per (m/s)
  public static final double LOOKAHEAD_MIN_SEC = 0.0;
  public static final double LOOKAHEAD_MAX_SEC = 1.5;
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

    public static final Pose2d LT_ENTER_POS = new Pose2d(5.848, 7.241, Rotation2d.fromDegrees(90));
    public static final Pose2d RT_ENTER_POS = new Pose2d(5.839, 0.823, Rotation2d.fromDegrees(-90));
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

  public static class LimelightConstants {
    public static final String LIMELIGHT_RIGHT = "limelight-right";//10.58.29.17
    public static final String LIMELIGHT_BACK = "limelight-back"; //10.58.29.16
    public static final String LIMELIGHT_LEFT = "limelight-left"; //10.58.29.15
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // INTAKE
  public static class IntakeConstants {
    public static final int INTAKE_ID = 19;

    // PID Constants
    public static final double p = 0.0;
    public static final double i = 0.0;
    public static final double d = 0.0;

    public static final double s = 0.0;
    public static final double v = 0.0;
    public static final double a = 0.0;

    public static final double INTAKE_DUTY = 0.8;
    public static final double OUTTAKE_DUTY = -0.8;

  }
  public static class SlapdownConstants {
    public static final int SLAPDOWN_ID = 18;

    // PID Constants
    public static final double slowP = 1.0;
    public static final double slowI = 0.0;
    public static final double slowD = 0.0;

    public static final double fastP = 2.0;
    public static final double fastI = 0.0;
    public static final double fastD = 0.1;

    public static final double maxout = 25;
  }

  // KICKER
  public static class KickerConstants {
    public static final int KICKER_LEFT_ID = 13;
    public static final int KICKER_RIGHT_ID = 14;

    // PID Constants
    public static final double p = 0.001;
    public static final double i = 0.0;
    public static final double d = 0.0;

    // Feed-Forward Constants
    public static final double s = 0.0;
    public static final double v = 0.0;
    public static final double a = 0.0;

    public static final double KICKER_RPM = 1600;
    public static final double KICKER_REVERSE_RPM = -1600;

  }
    public static class HoodConstants {
      public static final int HOOD_ID = 15;

      public static final double HOOD_MIN = 0.0;
      public static final double HOOD_MAX = 2.907;

      public static final double POSITION_TOLERANCE_ROTATIONS = 0.05;

      public static final double HOOD_MIN_DEGREES = 22.5279443;
      public static final double HOOD_MAX_DEGREES = 45.334706;

      public static final double DEGREES_PER_ROTATION =
          (HOOD_MAX_DEGREES - HOOD_MIN_DEGREES) / (HOOD_MAX - HOOD_MIN);

      public static double rotationsToDegrees(double rotations) {
        return HOOD_MIN_DEGREES + (rotations - HOOD_MIN) * DEGREES_PER_ROTATION;
      }

      public static double degreesToRotations(double degrees) {
        return HOOD_MIN + (degrees - HOOD_MIN_DEGREES) / DEGREES_PER_ROTATION;
      }

      public static double clampRotations(double rotations) {
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, rotations));
      }

      public static final InterpolatingDoubleTreeMap hubHoodTable = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap ferryHoodTable = new InterpolatingDoubleTreeMap();

      static {
        // double tuningRatio = 0.5;
        // double min = HOOD_MIN_DEGREES;
        // double max = HOOD_MAX_DEGREES;
        // for (var entry : List.of(
        //     Pair.of(Meters.of(1.4),  Degrees.of(min + (0 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(1.6),  Degrees.of(min + (0 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(1.8),  Degrees.of(min + (0 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(2.0),  Degrees.of(min + (3.05 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(2.25), Degrees.of(min + (6.60 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(2.5),  Degrees.of(min + (8.7 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(3.0),  Degrees.of(min + (11.2 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(3.5),  Degrees.of(min + (13.1 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(4.0),  Degrees.of(min + (14.65 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(4.5),  Degrees.of(min + (15.9 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(5.0),  Degrees.of(min + (16.9 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(5.5),  Degrees.of(min + (17.8 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(6.0),  Degrees.of(min + (18.6 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(6.5),  Degrees.of(min + (19.25 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(7.0),  Degrees.of(min + (19.9 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(7.5),  Degrees.of(min + (20.45 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(8.0),  Degrees.of(min + (20.95 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(8.5),  Degrees.of(min + (21.45 * (tuningRatio / (max - min))))),
        //     Pair.of(Meters.of(9.0),  Degrees.of(min + (21.9 * (tuningRatio / (max - min))))))) {
        //     hubHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        // }
        
        double tuningAngle = 0.5;
        for (var entry : List.of(
          Pair.of(Meters.of(1.4), Degrees.of(22.53)),
          Pair.of(Meters.of(1.6), Degrees.of(22.53)),
          Pair.of(Meters.of(1.8), Degrees.of(22.53)),
          Pair.of(Meters.of(2.0), Degrees.of(25.58-tuningAngle)),
          Pair.of(Meters.of(2.25), Degrees.of(29.13-tuningAngle)),
          Pair.of(Meters.of(2.5), Degrees.of(31.23-tuningAngle)),
          Pair.of(Meters.of(3.0), Degrees.of(33.73-tuningAngle)),
          Pair.of(Meters.of(3.5), Degrees.of(35.63-tuningAngle)),
          Pair.of(Meters.of(4.0), Degrees.of(37.18-tuningAngle)),
          Pair.of(Meters.of(4.5), Degrees.of(38.43-tuningAngle)),
          Pair.of(Meters.of(5.0), Degrees.of(39.43-tuningAngle)),
          Pair.of(Meters.of(5.5), Degrees.of(40.33-tuningAngle)),
          Pair.of(Meters.of(6.0), Degrees.of(41.13-tuningAngle)),
          Pair.of(Meters.of(6.5), Degrees.of(41.78-tuningAngle)),
          Pair.of(Meters.of(7.0), Degrees.of(42.43-tuningAngle)),
          Pair.of(Meters.of(7.5), Degrees.of(42.98-tuningAngle)),
          Pair.of(Meters.of(8.0), Degrees.of(43.48-tuningAngle)),
          Pair.of(Meters.of(8.5), Degrees.of(43.98-tuningAngle)),
          Pair.of(Meters.of(9.0), Degrees.of(44.43-tuningAngle)))) {
          hubHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }

        for (var entry : List.of(
          Pair.of(Meters.of(2.0), Degrees.of(45.33)),
          Pair.of(Meters.of(2.5), Degrees.of(45.33)),
          Pair.of(Meters.of(3.0), Degrees.of(45.33)),
          Pair.of(Meters.of(4.0), Degrees.of(45.33)),
          Pair.of(Meters.of(5.0), Degrees.of(45.33)),
          Pair.of(Meters.of(6.0), Degrees.of(45.33)),
          Pair.of(Meters.of(7.0), Degrees.of(45.33)),
          Pair.of(Meters.of(8.0), Degrees.of(45.33)),
          Pair.of(Meters.of(9.0), Degrees.of(45.33)),
          Pair.of(Meters.of(10.0), Degrees.of(45.33)))) {
          ferryHoodTable.put(entry.getFirst().in(Meters), entry.getSecond().in(Degrees));
        }
      }

      public static final double HOOD_DOWN = 0.0;
      public static final double HOOD_UP = 2.8;

      public static final double TRENCH_BUFFER_M = 0.80;
      public static final double TRENCH_LOOKAHEAD_S = 0.5;

      // PID Constants
      public static final double p = 1.4;
      public static final double i = 0.0;
      public static final double d = 0.0;
    }
    public static class ShooterConstants {

      public static final int SHOOTER_R1_ID = 9;
      public static final int SHOOTER_L1_ID = 10;
      public static final int SHOOTER_R2_ID = 11; 
      public static final int SHOOTER_L2_ID = 12;

      public static final double ERROR_MARGIN = 100.0;
      
       
      // PID Constants
      public static final double p = 0.001;
      public static final double i = 0.0;
      public static final double d = 0.0;

      // Feed-Forward Constants
      public static final double s = 0.0;
      public static final double v = 0.001935;
      public static final double a = 0.0;

      public static final double MIN_HUB_DISTANCE_M = 1.4;
      public static final double MAX_HUB_DISTANCE_M = 9.0;
      public static final double MIN_FERRY_DISTANCE_M = 2.0;
      public static final double MAX_FERRY_DISTANCE_M = 10.0;

      public static final double MIN_HUB_SHOT_DISTANCE_M = 1.15;
      public static final double HUB_AIM_TOLERANCE_M = 0.35;
      public static final double FERRY_AIM_TOLERANCE_M = 1.0;

      public static final double ALLIANCE_IDLE_RPM = 1500.0;

      public static final double ROLLER_RADIUS_BOTTOM_M = 2.0 * 0.0254;
      public static final double ROLLER_RADIUS_TOP_M = 0.625 * 0.0254;
      public static final double PULLEY_TOP_PER_BOTTOM = 1.0;

      public static final double FUEL_MASS_KG = 0.215;
      public static final double DRAG_COEFFICIENT = 0.50;
      public static final double AIR_DENSITY_KG_PER_M3 = 1.204;

      public final static InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();
      static {
        for (var entry : List.of(
          Pair.of(Meters.of(1.4), Seconds.of(0.575)),
          Pair.of(Meters.of(1.6), Seconds.of(0.660)),
          Pair.of(Meters.of(1.8), Seconds.of(0.737)),
          Pair.of(Meters.of(2.0), Seconds.of(0.723)),
          Pair.of(Meters.of(2.25), Seconds.of(0.712)),
          Pair.of(Meters.of(2.5), Seconds.of(0.731)),
          Pair.of(Meters.of(3.0), Seconds.of(0.792)),
          Pair.of(Meters.of(3.5), Seconds.of(0.851)),
          Pair.of(Meters.of(4.0), Seconds.of(0.906)),
          Pair.of(Meters.of(4.5), Seconds.of(0.960)),
          Pair.of(Meters.of(5.0), Seconds.of(1.012)),
          Pair.of(Meters.of(5.5), Seconds.of(1.062)),
          Pair.of(Meters.of(6.0), Seconds.of(1.109)),
          Pair.of(Meters.of(6.5), Seconds.of(1.157)),
          Pair.of(Meters.of(7.0), Seconds.of(1.201)),
          Pair.of(Meters.of(7.5), Seconds.of(1.245)),
          Pair.of(Meters.of(8.0), Seconds.of(1.288)),
          Pair.of(Meters.of(8.5), Seconds.of(1.329)),
          Pair.of(Meters.of(9.0), Seconds.of(1.369)))) {
          TOF.put(entry.getFirst().in(Meters), entry.getSecond().in(Seconds));
        }
      }

      public final static double RPM_SCALE = 1.0; 

      public final static InterpolatingDoubleTreeMap ferryTOF = new InterpolatingDoubleTreeMap();
      static {
        for (var entry : List.of(
          Pair.of(Meters.of(2.0), Seconds.of(0.689)),
          Pair.of(Meters.of(2.5), Seconds.of(0.764)),
          Pair.of(Meters.of(3.0), Seconds.of(0.833)),
          Pair.of(Meters.of(4.0), Seconds.of(0.960)),
          Pair.of(Meters.of(5.0), Seconds.of(1.076)),
          Pair.of(Meters.of(6.0), Seconds.of(1.184)),
          Pair.of(Meters.of(7.0), Seconds.of(1.287)),
          Pair.of(Meters.of(8.0), Seconds.of(1.386)),
          Pair.of(Meters.of(9.0), Seconds.of(1.481)),
          Pair.of(Meters.of(10.0), Seconds.of(1.574)))) {
          ferryTOF.put(entry.getFirst().in(Meters), entry.getSecond().in(Seconds));
        }
      }

      public static final InterpolatingDoubleTreeMap hubShooterTable = new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap ferryShooterTable = new InterpolatingDoubleTreeMap();
      static {

        for (var entry : List.of(
          Pair.of(Meters.of(1.4), RPM.of(1605*RPM_SCALE)),
          Pair.of(Meters.of(1.6), RPM.of(1645*RPM_SCALE)),
          Pair.of(Meters.of(1.8), RPM.of(1700*RPM_SCALE)),
          Pair.of(Meters.of(2.0), RPM.of(1730*RPM_SCALE)),
          Pair.of(Meters.of(2.25), RPM.of(1775*RPM_SCALE)),
          Pair.of(Meters.of(2.5), RPM.of(1825*RPM_SCALE)),
          Pair.of(Meters.of(3.0), RPM.of(1935*RPM_SCALE)),
          Pair.of(Meters.of(3.5), RPM.of(2040*RPM_SCALE)),
          Pair.of(Meters.of(4.0), RPM.of(2145*RPM_SCALE)),
          Pair.of(Meters.of(4.5), RPM.of(2245*RPM_SCALE)),
          Pair.of(Meters.of(5.0), RPM.of(2345*RPM_SCALE)),
          Pair.of(Meters.of(5.5), RPM.of(2445*RPM_SCALE)),
          Pair.of(Meters.of(6.0), RPM.of(2540*RPM_SCALE)),
          Pair.of(Meters.of(6.5), RPM.of(2635*RPM_SCALE)),
          Pair.of(Meters.of(7.0), RPM.of(2725*RPM_SCALE)),
          Pair.of(Meters.of(7.5), RPM.of(2820*RPM_SCALE)),
          Pair.of(Meters.of(8.0), RPM.of(2910*RPM_SCALE)),
          Pair.of(Meters.of(8.5), RPM.of(2995*RPM_SCALE)),
          Pair.of(Meters.of(9.0), RPM.of(3085*RPM_SCALE)))) {
          hubShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
        }

        for (var entry : List.of(
          Pair.of(Meters.of(2.0), RPM.of(1065*RPM_SCALE)),
          Pair.of(Meters.of(2.5), RPM.of(1240*RPM_SCALE)),
          Pair.of(Meters.of(3.0), RPM.of(1400*RPM_SCALE)),
          Pair.of(Meters.of(4.0), RPM.of(1680*RPM_SCALE)),
          Pair.of(Meters.of(5.0), RPM.of(1930*RPM_SCALE)),
          Pair.of(Meters.of(6.0), RPM.of(2160*RPM_SCALE)),
          Pair.of(Meters.of(7.0), RPM.of(2375*RPM_SCALE)),
          Pair.of(Meters.of(8.0), RPM.of(2575*RPM_SCALE)),
          Pair.of(Meters.of(9.0), RPM.of(2770*RPM_SCALE)),
          Pair.of(Meters.of(10.0), RPM.of(2955*RPM_SCALE)))) {
          ferryShooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
        }
      }
    }

    public static class HopperConstants {
      public static final int BELTS_LEFT_ID = 16;
      public static final int BELTS_RIGHT_ID = 17;

      public static final double BELTS_SPEED = 0.75;
      public static final double REVERSE_BELTS_SPEED = -0.75;
    
      // PID Constants
      public static final double p = 0.0;
      public static final double i = 0.0;
      public static final double d = 0.0;

      // Feed-Forward Constants
      public static final double s = 0.0;
      public static final double v = 0.0;
      public static final double a = 0.0;

    }
    public static final double X_REEF_ALIGNMENT_P = 2.1; // Proportional gain for X-axis reef alignment
    public static final double Y_REEF_ALIGNMENT_P = 2.5; // Proportional gain for Y-axis reef alignment (previously
                                                         // 1.74)
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
      public static final Distance BUMPER_THICKNESS = Inches.of(3.82); // frame to edge of bumper
      public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
      public static final Distance FRAME_SIZE_Y = Inches.of(27); // left to right (y-axis)
      public static final Distance FRAME_SIZE_X = Inches.of(27); // front to back (x-axis)

      public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
      public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    }

  
}