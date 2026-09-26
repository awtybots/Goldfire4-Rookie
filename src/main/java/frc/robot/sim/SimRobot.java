package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

public class SimRobot {

  public static final class SimConstants {
    public static final double BUMPER_LENGTH_M = 0.880;
    public static final double BUMPER_WIDTH_M = 0.880;
    public static final double BUMPER_HEIGHT_M = Dimensions.BUMPER_HEIGHT.in(Meters);

    public static final int FUEL_CAPACITY = 50;
    public static final double FUEL_RADIUS_M = FieldConstants.fuelDiameter / 2.0;

    public static final double INTAKE_REACH_M = 0.15;
    public static final double INTAKE_HALF_WIDTH_M = 0.32;

    public static final double EXIT_X_M = -Units.inchesToMeters(9.014);
    public static final double EXIT_Z_M = Units.inchesToMeters(20.313);
    public static final double[] LANE_Y_M = {-0.225, -0.075, 0.075, 0.225};
    public static final double LAUNCH_YAW_OFFSET_DEG = 180.0;
    public static final double VOLLEYS_PER_SECOND = 3.0; // only used by launchVolley()
    public static final double RELEASE_STAGGER_S = 0.06;
    // The feed is continuous now, so fuel leaves as an even stream rather than clumps of four.
    // 12/s is the same throughput the old 3 volleys x 4 lanes produced.
    public static final double BALLS_PER_SECOND = 12.0;
    public static final double RELEASE_JITTER = 0.15;
    public static final double SPEED_SIGMA = 0.02;
    public static final double LAUNCH_ANGLE_SIGMA_DEG = 0.8;
    public static final double YAW_SIGMA_DEG = 1.0;
    public static final double SPIN_SIGMA = 0.08;
    public static final double LANE_POSITION_SIGMA_M = 0.012;
    public static boolean FLYWHEEL_SAG = true;
    public static final double FUEL_MOI_FACTOR = 0.4;
    public static final long RANDOM_SEED = 5829L;

    public static final double HOPPER_X_MIN = -0.17;
    public static final double HOPPER_X_MAX = 0.336;
    public static final double HOPPER_Y_HALF = 0.346;
    public static final double HOPPER_Z_MIN = 0.070;
    public static final double HOPPER_Z_MAX = 0.521;

    public static final double GROUND_FRICTION_PER_SEC = 1.6;
    public static final double LOGGING_HZ = 50.0; // every loop: fuel positions must not stutter
    public static final int SUBTICKS = 10;

    public static final double FIELD_LENGTH_M = 16.51;
    public static final double FIELD_WIDTH_M = 8.04;

    public static final Pose2d START_POSE = new Pose2d(2.0, 4.02, Rotation2d.kZero);

    public static final double DRAG_K = 0.5 * ShooterConstants.AIR_DENSITY_KG_PER_M3
        * ShooterConstants.DRAG_COEFFICIENT * Math.PI * FUEL_RADIUS_M * FUEL_RADIUS_M
        / ShooterConstants.FUEL_MASS_KG;
    public static final double LIFT_K = 0.5 * ShooterConstants.AIR_DENSITY_KG_PER_M3
        * Math.PI * FUEL_RADIUS_M * FUEL_RADIUS_M / ShooterConstants.FUEL_MASS_KG;
  }

  public static final double[][] OBSTACLES = {
      {4.01, 5.21, 3.42, 4.62},
      {SimConstants.FIELD_LENGTH_M - 5.21, SimConstants.FIELD_LENGTH_M - 4.01, 3.42, 4.62},
      {0.95, 1.14, 3.28, 4.18},
      {SimConstants.FIELD_LENGTH_M - 1.14, SimConstants.FIELD_LENGTH_M - 0.95,
          SimConstants.FIELD_WIDTH_M - 4.18, SimConstants.FIELD_WIDTH_M - 3.28},
      {3.96, 5.18, 1.265, 1.570},
      {3.96, 5.18, SimConstants.FIELD_WIDTH_M - 1.570, SimConstants.FIELD_WIDTH_M - 1.265},
      {SimConstants.FIELD_LENGTH_M - 5.18, SimConstants.FIELD_LENGTH_M - 3.96, 1.265, 1.570},
      {SimConstants.FIELD_LENGTH_M - 5.18, SimConstants.FIELD_LENGTH_M - 3.96,
          SimConstants.FIELD_WIDTH_M - 1.570, SimConstants.FIELD_WIDTH_M - 1.265},
  };

  private final SwerveSubsystem drivebase;
  private final Shooter shooter;
  private final Hood hood;
  private final Kicker kicker;
  private final Intake intake;

  private final FuelSim fuelSim = new FuelSim("FuelSim");

  private int fuelStored = 0;
  private double nextReleaseTime = Double.NEGATIVE_INFINITY;
  private int laneCursor = 0;
  private final java.util.Random rng = new java.util.Random(SimConstants.RANDOM_SEED);
  private int hubShotsFired = 0;

  public SimRobot(SwerveSubsystem drivebase, Shooter shooter, Hood hood, Kicker kicker,
      Intake intake) {
    this.drivebase = drivebase;
    this.shooter = shooter;
    this.hood = hood;
    this.kicker = kicker;
    this.intake = intake;

    fuelSim.registerRobot(
        SimConstants.BUMPER_WIDTH_M,
        SimConstants.BUMPER_LENGTH_M,
        SimConstants.BUMPER_HEIGHT_M,
        drivebase::getPose,
        drivebase::getFieldVelocity);

    fuelSim.registerIntake(
        SimConstants.BUMPER_LENGTH_M / 2.0,
        SimConstants.BUMPER_LENGTH_M / 2.0 + SimConstants.INTAKE_REACH_M,
        -SimConstants.INTAKE_HALF_WIDTH_M,
        SimConstants.INTAKE_HALF_WIDTH_M,
        this::canIntake,
        this::onFuelIntaked);

    fuelSim.setGroundFriction(SimConstants.GROUND_FRICTION_PER_SEC);
    fuelSim.useQuadraticDragWithMagnus(SimConstants.DRAG_K, SimConstants.LIFT_K);
    fuelSim.setSubticks(SimConstants.SUBTICKS);
    fuelSim.setLoggingFrequency(SimConstants.LOGGING_HZ);
    fuelSim.spawnStartingFuel();
    fuelSim.start();

    SmartDashboard.putData(Commands.runOnce(this::resetFuel)
        .withName("Reset Fuel").ignoringDisable(true));
    SmartDashboard.putData(Commands.runOnce(this::preloadFuel)
        .withName("Preload Fuel").ignoringDisable(true));
    SmartDashboard.putData(Commands.runOnce(() -> drivebase.resetOdometry(SimConstants.START_POSE))
        .withName("Reset Robot Pose").ignoringDisable(true));

    drivebase.resetOdometry(SimConstants.START_POSE);
  }

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  public int getFuelStored() {
    return fuelStored;
  }

  public int getHubShotsFired() {
    return hubShotsFired;
  }

  public void resetFuel() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
    fuelStored = 0;
    hubShotsFired = 0;
    nextReleaseTime = Double.NEGATIVE_INFINITY;
    laneCursor = 0;
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
  }

  public void preloadFuel() {
    fuelStored = SimConstants.FUEL_CAPACITY;
  }

  private boolean canIntake() {
    return fuelStored < SimConstants.FUEL_CAPACITY && intake.isIntaking();
  }

  private void onFuelIntaked() {
    fuelStored++;
  }

  public static double ballSpeedMetersPerSecond(double motorRPM) {
    double omega = motorRPM * 2.0 * Math.PI / 60.0;
    return omega * (ShooterConstants.ROLLER_RADIUS_BOTTOM_M
        + ShooterConstants.PULLEY_TOP_PER_BOTTOM * ShooterConstants.ROLLER_RADIUS_TOP_M) / 2.0;
  }

  public static double backspinRadPerSec(double motorRPM) {
    double omega = motorRPM * 2.0 * Math.PI / 60.0;
    return omega * (ShooterConstants.ROLLER_RADIUS_BOTTOM_M
        - ShooterConstants.PULLEY_TOP_PER_BOTTOM * ShooterConstants.ROLLER_RADIUS_TOP_M)
        / (2.0 * SimConstants.FUEL_RADIUS_M);
  }

  public static double launchAngleDegrees(double hoodDegrees) {
    return 90.0 - hoodDegrees;
  }

  private static FuelSim.Hub ourHub() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? FuelSim.Hub.RED_HUB : FuelSim.Hub.BLUE_HUB;
  }

  public void periodic() {
    updateVolleys();
    fuelSim.updateSim();

    int scored = ourHub().getScore();
    Logger.recordOutput("Sim/FuelStored", fuelStored);
    Logger.recordOutput("Sim/HeldFuel", heldFuelPositions());
    Logger.recordOutput("Sim/FuelOnField", fuelSim.getFuelCount());
    Logger.recordOutput("Sim/FuelInFlight", fuelSim.getFuelInFlightCount());
    Logger.recordOutput("Sim/ShooterRPM", shooter.getRPM());
    Logger.recordOutput("Sim/ShooterTargetRPM", shooter.getTargetRPM());
    Logger.recordOutput("Sim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("Sim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
    Logger.recordOutput("Sim/BallSpeed", ballSpeedMetersPerSecond(shooter.getRPM()));
    Logger.recordOutput("Sim/LaunchAngleDeg", launchAngleDegrees(hood.getAngleDegrees()));
    Logger.recordOutput("Sim/HubShotsFired", hubShotsFired);
    Logger.recordOutput("Sim/HubShotsScored", scored);
    Logger.recordOutput("Sim/HubAccuracyPct",
        hubShotsFired == 0 ? 0.0 : 100.0 * scored / hubShotsFired);
  }

  private void updateVolleys() {
    double now = Timer.getFPGATimestamp();

    if (!kicker.isFeeding() || fuelStored <= 0) {
      nextReleaseTime = Double.NEGATIVE_INFINITY; // first ball of a burst leaves immediately
      return;
    }

    if (nextReleaseTime == Double.NEGATIVE_INFINITY) {
      nextReleaseTime = now;
    }

    // Release on a fixed cadence, catching up if a loop ran long, so the stream stays even
    // instead of arriving in clumps.
    double gap = 1.0 / SimConstants.BALLS_PER_SECOND;
    while (fuelStored > 0 && nextReleaseTime <= now) {
      launchBall(SimConstants.LANE_Y_M.length == 0 ? 0 : laneCursor % SimConstants.LANE_Y_M.length);
      laneCursor++;
      nextReleaseTime += gap * (1.0 + gaussian(SimConstants.RELEASE_JITTER));
    }
  }

  public void launchVolley() {
    int count = Math.min(SimConstants.LANE_Y_M.length, fuelStored);
    for (int lane = 0; lane < count; lane++) {
      launchBall(lane);
    }
  }

  private double gaussian(double sigma) {
    return rng.nextGaussian() * sigma;
  }

  private void launchBall(int lane) {
    Pose2d pose = drivebase.getPose();
    ChassisSpeeds field = drivebase.getFieldVelocity();
    double rpm = shooter.getRPM();

    double speed = ballSpeedMetersPerSecond(rpm) * (1.0 + gaussian(SimConstants.SPEED_SIGMA));
    double spin = backspinRadPerSec(rpm) * (1.0 + gaussian(SimConstants.SPIN_SIGMA));
    double elevation = Math.toRadians(launchAngleDegrees(hood.getAngleDegrees())
        + gaussian(SimConstants.LAUNCH_ANGLE_SIGMA_DEG));
    double yaw = pose.getRotation().getRadians()
        + Math.toRadians(SimConstants.LAUNCH_YAW_OFFSET_DEG + gaussian(SimConstants.YAW_SIGMA_DEG));

    Translation2d offset = new Translation2d(SimConstants.EXIT_X_M,
        SimConstants.LANE_Y_M[lane] + gaussian(SimConstants.LANE_POSITION_SIGMA_M))
        .rotateBy(pose.getRotation());
    Translation2d exit = offset.plus(pose.getTranslation());
    double omega = field.omegaRadiansPerSecond;
    double horizontal = speed * Math.cos(elevation);

    fuelSim.spawnFuel(
        new Translation3d(exit.getX(), exit.getY(), SimConstants.EXIT_Z_M),
        new Translation3d(
            horizontal * Math.cos(yaw) + field.vxMetersPerSecond - omega * offset.getY(),
            horizontal * Math.sin(yaw) + field.vyMetersPerSecond + omega * offset.getX(),
            speed * Math.sin(elevation)),
        spin);

    if (SimConstants.FLYWHEEL_SAG) {
      double mass = ShooterConstants.FUEL_MASS_KG;
      double ballMoi = SimConstants.FUEL_MOI_FACTOR * mass
          * SimConstants.FUEL_RADIUS_M * SimConstants.FUEL_RADIUS_M;
      shooter.drawFlywheelEnergy(0.5 * mass * speed * speed + 0.5 * ballMoi * spin * spin);
    }

    fuelStored--;
    if (drivebase.isHubShot()) {
      hubShotsFired++;
    }
  }

  public Translation3d[] heldFuelPositions() {
    if (fuelStored <= 0) {
      return new Translation3d[0];
    }
    double r = SimConstants.FUEL_RADIUS_M;
    double xLo = SimConstants.HOPPER_X_MIN + r;
    double xHi = SimConstants.HOPPER_X_MAX - r;
    double yLo = -SimConstants.HOPPER_Y_HALF + r;
    double yHi = SimConstants.HOPPER_Y_HALF - r;
    double zLo = SimConstants.HOPPER_Z_MIN + r;
    double zHi = SimConstants.HOPPER_Z_MAX - r;

    int nx = Math.max(1, (int) Math.floor((xHi - xLo) / (2.0 * r)) + 1);
    int ny = Math.max(1, (int) Math.floor((yHi - yLo) / (2.0 * r)) + 1);
    int perLayer = nx * ny;
    int layers = Math.max(1, (int) Math.ceil((double) fuelStored / perLayer));

    Pose2d pose = drivebase.getPose();
    Translation3d[] out = new Translation3d[fuelStored];
    for (int i = 0; i < fuelStored; i++) {
      int layer = i / perLayer;
      int slot = i % perLayer;
      int ix = slot % nx;
      int iy = slot / nx;
      double x = nx == 1 ? (xLo + xHi) / 2.0 : xLo + ix * (xHi - xLo) / (nx - 1);
      double y = ny == 1 ? (yLo + yHi) / 2.0 : yLo + iy * (yHi - yLo) / (ny - 1);
      double z = layers == 1 ? zLo : zLo + layer * (zHi - zLo) / (layers - 1);
      Translation2d inField = new Translation2d(x, y).rotateBy(pose.getRotation());
      out[i] = new Translation3d(pose.getX() + inField.getX(), pose.getY() + inField.getY(), z);
    }
    return out;
  }
}
