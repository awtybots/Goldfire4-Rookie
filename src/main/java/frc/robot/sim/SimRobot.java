package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public static final double VOLLEYS_PER_SECOND = 3.0;

    public static final double HOPPER_X_MIN = -0.17;
    public static final double HOPPER_X_MAX = 0.336;
    public static final double HOPPER_Y_HALF = 0.346;
    public static final double HOPPER_Z_MIN = 0.070;
    public static final double HOPPER_Z_MAX = 0.521;

    public static final double GROUND_FRICTION_PER_SEC = 1.6;
    public static final double LOGGING_HZ = 15.0;
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
  private double volleyAccumulator = 0.0;
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
    volleyAccumulator = 0.0;
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
    if (!kicker.isFeeding() || fuelStored <= 0) {
      volleyAccumulator = 0.0;
      return;
    }
    volleyAccumulator += 0.020 * SimConstants.VOLLEYS_PER_SECOND;
    while (volleyAccumulator >= 1.0 && fuelStored > 0) {
      volleyAccumulator -= 1.0;
      launchVolley();
    }
  }

  public void launchVolley() {
    int count = Math.min(SimConstants.LANE_Y_M.length, fuelStored);
    if (count <= 0) {
      return;
    }

    Pose2d pose = drivebase.getPose();
    ChassisSpeeds field = drivebase.getFieldVelocity();
    double rpm = shooter.getRPM();
    double speed = ballSpeedMetersPerSecond(rpm);
    double spin = backspinRadPerSec(rpm);
    double elevation = Math.toRadians(launchAngleDegrees(hood.getAngleDegrees()));
    double yaw = pose.getRotation().getRadians() + Math.toRadians(SimConstants.LAUNCH_YAW_OFFSET_DEG);
    double horizontal = speed * Math.cos(elevation);

    for (int i = 0; i < count; i++) {
      int lane = SimConstants.LANE_Y_M.length == count ? i
          : (SimConstants.LANE_Y_M.length - count) / 2 + i;
      Translation2d exit = new Translation2d(SimConstants.EXIT_X_M, SimConstants.LANE_Y_M[lane])
          .rotateBy(pose.getRotation())
          .plus(pose.getTranslation());
      fuelSim.spawnFuel(
          new Translation3d(exit.getX(), exit.getY(), SimConstants.EXIT_Z_M),
          new Translation3d(
              horizontal * Math.cos(yaw) + field.vxMetersPerSecond,
              horizontal * Math.sin(yaw) + field.vyMetersPerSecond,
              speed * Math.sin(elevation)),
          spin);
    }

    fuelStored -= count;
    if (drivebase.isInAllianceZone()) {
      hubShotsFired += count;
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
