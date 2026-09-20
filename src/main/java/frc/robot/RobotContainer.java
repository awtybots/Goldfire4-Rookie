// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// teaching

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.AimHood;
import frc.robot.commands.ControlAllShooting;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.DrivebaseConstants;
// import frc.robot.Configs.ShooterSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.HoodConstants;
// import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.HubTracker;
// import frc.robot.utils.FuelSim;

import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.BooleanSupplier;

// import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.HubTrackerSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Slapdown;
import frc.robot.subsystems.ObjectDetection;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Instantiate Subsystems
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();
  private final Hood m_Hood = new Hood(drivebase::isNearTrench);
  // private final Climber m_climber = new Climber();
  private final Kicker m_kicker = new Kicker();
  private final Slapdown m_slapdown = new Slapdown();

  // Helper Subsystems
  private final HubTrackerSubsystem m_hubtracker = new HubTrackerSubsystem(drivebase, driverXbox);

  // private final ObjectDetection m_ObjectDetection = new ObjectDetection();

  // Factory for ControlAllShooting instances. Create a fresh instance for each
  // composition to avoid WPILib's "composed commands may not be reused" error.
  // private ControlAllShooting makeVariableShoot() {
  // return new ControlAllShooting(drivebase::getCachedDynamicHubLocation,
  // m_shooter, drivebase::getPose);
  // }

  // public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of
  // FuelSim

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> loggedAutoChooser;
  // Add this field at the top of RobotContainer (alongside your other fields)
  private SendableChooser<Boolean> flipChooser = new SendableChooser<>();


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity;

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle;

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented;

  SwerveInputStream driveAngularVelocityKeyboard;
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard;

  // Factory for ControlAllShooting instances. Create a fresh instance for each
  // composition to avoid WPILib's "composed commands may not be reused" error.
  private ControlAllShooting makeVariableShoot() {
    return new ControlAllShooting(m_shooter, m_hopper, m_kicker, m_Hood, drivebase);
  }

  // Factory for AimHood instances, same fresh-instance rule as above.
  // Hub mode uses the hub LUT, ferry mode uses the ferry LUT.
  private AimHood makeAimHoodHub() {
    return new AimHood(m_Hood, drivebase::getCachedDynamicHubLocation, drivebase::getPose, false);
  }

  private AimHood makeAimHoodFerry() {
    return new AimHood(m_Hood, drivebase::getCachedDynamicFerryLocation, drivebase::getPose, true);
  }

  private Command makeAutoShoot() {
    return Commands.parallel(
        new AimAtTarget(drivebase, autoAimStream, () -> 0.0, () -> 0.0),
        Commands.defer(() -> {
          if (drivebase.isInAllianceZone()) { // In alliance zone -> shoot at hub
            return Commands.parallel(
                makeVariableShoot(),
                makeAimHoodHub(),
                m_slapdown.slowretractCommand().beforeStarting(Commands.waitSeconds(3)));
          } else {
            return Commands.parallel(
                makeVariableShoot(),
                makeAimHoodFerry(),
                m_slapdown.slowretractCommand().beforeStarting(Commands.waitSeconds(3)));
          }
        }, Set.of(m_shooter, m_hopper, m_kicker, m_Hood, m_slapdown)))
        .withTimeout(5);
  }

  /** Returns the controller that should be treated as the driving controller. */
  private CommandXboxController dc() {
      return driverXbox;
  }

  private CommandXboxController oc() {
      return operatorXbox;
  }

  private AimAtTarget aimAtTarget;
  private PathConstraints autoConstraints;

  SwerveInputStream aimStream;
  SwerveInputStream autoAimStream;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // configureFuelSim();
    // configureFuelSimRobot();
    // Triggers for auto aim/pass poses

    DriverStation.silenceJoystickConnectionWarning(true);
    // SmartDashboard.putBoolean("Is Shooter Running",
    // m_shooter.isShooterRunning());
    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("extend", m_slapdown.extendCommand());
    NamedCommands.registerCommand("Intake", m_intake.runIntakeCommand());
    NamedCommands.registerCommand("shoot", makeAutoShoot());

    // setup the flip chooser
    flipChooser.setDefaultOption("Not Flipped", false);
    flipChooser.addOption("Flipped", true);
    SmartDashboard.putData("Flip Auto", flipChooser);

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
  }

  /**
   * Constructs throwaway instances of the commands that fire from deferred RT
   * bindings
   * so first-use class loading (WPILib units system, InterpolatingDoubleTreeMap,
   * SwerveInputStream.copy, command composition) happens at robot boot instead of
   * mid-match. Nothing is scheduled — zero runtime side effects. Side-effect-free
   * because ControlAllShooting's no-arg-requireShooter overload skips
   * addRequirements,
   * and the command constructors only assign fields / copy the input stream.
   */

  public void warmupCommands() {
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)
        .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(1.0)
        .allianceRelativeControl(true);

    /*
     * driverXbox.rightTrigger().whileTrue(Commands.defer(() -> {
     * if (isInAllianceZone()) {
     * aimAtHub = new AimAtHub(drivebase, driveAngularVelocity,
     * driverXbox::getLeftX, driverXbox::getLeftY, driverXbox::getRightX);
     * return aimAtHub;
     * } else {
     * aimAtFerry = new AimAtFerry(drivebase, driveAngularVelocity);
     * return aimAtFerry;
     * }
     * }, Set.of(drivebase)));
     */
    driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
        .headingWhile(true);

    driveRobotOriented = driveAngularVelocity.copy()
        .robotRelative(true)
        .allianceRelativeControl(false);

    driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // Derive the heading axis with math!
    driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(
            () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees(0));

    aimStream = driveAngularVelocity.copy();

    autoAimStream = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0)
        .allianceRelativeControl(true);


    aimAtTarget = new AimAtTarget(drivebase, aimStream,
        dc()::getLeftX, dc()::getLeftY);


    dc().rightTrigger().whileTrue(aimAtTarget);


    dc().rightTrigger().whileTrue(
        Commands.defer(() -> {
          if (drivebase.isInAllianceZone()) { // In alliance zone -> shoot at hub
            return Commands.parallel(
                makeVariableShoot(),
                makeAimHoodHub(),
                m_slapdown.slowretractCommand().beforeStarting(Commands.waitSeconds(3)));
          } else {
            return Commands.parallel(
                makeVariableShoot(),
                makeAimHoodFerry(),
                m_slapdown.slowretractCommand().beforeStarting(Commands.waitSeconds(3)));
          }
        }, Set.of(m_shooter, m_hopper, m_kicker, m_Hood, m_slapdown)));

    // ======== Operator ========
    // // shooter
    // dc().rightTrigger().whileTrue(
    //     Commands.parallel(
    //         m_shooter.setShooterSpeedCommand(1000),
    //         m_Hood.setHoodPositionCommand(0.6),
    //         Commands.sequence(
    //             Commands.waitUntil(() -> m_shooter.isShooterFast()),
    //             Commands.parallel(
    //                 m_kicker.kickCommand(),
    //                 m_slapdown.slowretractCommand().beforeStarting(Commands.waitSeconds(3)),
    //                 m_hopper.runBeltsToConveyorCommand()))));

    dc().rightBumper().whileTrue(
      Commands.parallel(
        m_intake.runOuttakeCommand(),
        m_hopper.runReverseBeltsCommand(),
        m_kicker.backwardsKickCommand()
      )
    );

    dc().leftTrigger().whileTrue(
      Commands.either(
        Commands.parallel(
            m_slapdown.extendCommand(),
            m_intake.runIntakeCommand())
        ,
        Commands.parallel(
            m_slapdown.extendCommand(),
            m_intake.runIntakeCommand().beforeStarting(Commands.waitSeconds(0.7)))
        ,
        m_slapdown::isSlapdownOut
    ));

    dc().leftBumper().whileTrue(
        (m_slapdown.retractCommand()));

    dc().a().whileTrue(m_hopper.runBeltsToConveyorCommand());

    // hood manual controls for testing/tuning
    oc().povUp().whileTrue(m_Hood.raiseHoodCommand());
    oc().povDown().whileTrue(m_Hood.lowerHoodCommand());
    // operatorXbox.rightTrigger().whileTrue(m_shooter.setShooterSpeedCommand(1200));
    // POVDOWN_OP_HoodDown.whileTrue(m_Hood.setHoodPositionCommand(HoodConstants.HOOD_DOWN));

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    // ====================================== ALIGN TO HUB COMMANDS
    // ======================================
    // ====================================== ALL CONTROLS
    // ======================================

    // ======= Driver =======

    // Swerve Drive Commands
    dc().start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // A_runOuttake.whileTrue(drivebase.lockCommand(
    // driverXbox::getLeftX,
    // driverXbox::getLeftY,
    // driverXbox::getRightX,
    // driveAngularVelocity::get)
    // .onlyWhile(autoAimCommand.swerveInputStream.aimLock(Angle.ofBaseUnits(3,
    // Degrees))));

    // ========================

    // SysId: run shooter quasistatic forward.
    // operatorXbox.a().whileTrue(m_shooter.sysIdQuasistaticForward());
    // // SysId: run shooter quasistatic reverse.
    // operatorXbox.b().whileTrue(m_shooter.sysIdQuasistaticReverse());
    // // SysId: run shooter dynamic forward.
    // operatorXbox.x().whileTrue(m_shooter.sysIdDynamicForward());
    // // SysId: run shooter dynamic reverse.
    // operatorXbox.y().whileTrue(m_shooter.sysIdDynamicReverse());

    // new Trigger(() -> isInAllianceZone()
    // && DriverStation.isTeleop())
    // .onTrue(Commands.runOnce(() ->
    // m_shooter.setDefaultCommand(m_shooter.setAllianceIdle())));
    // new Trigger(() -> !isInAllianceZone()
    // && DriverStation.isTeleopEnabled())
    // .onTrue(Commands.runOnce(() ->
    // m_shooter.setDefaultCommand(m_shooter.setNeutralIdle())));
    // m_shooter.setDefaultCommand(m_shooter.setAllianceIdle().onlyWhile(() ->
    // DriverStation.isTeleopEnabled()));

    // m_intake.setDefaultCommand(m_intake.runDefaultCommand());
    // m_kicker.setDefaultCommand(m_kicker.runDefaultCommand());
    // // m_slapdown.setDefaultCommand(m_slapdown.runDefaultCommand());
    // m_hopper.setDefaultCommand(m_hopper.runDefaultCommand());

 
    m_Hood.setDefaultCommand(m_Hood.tuckCommand());

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      if (Constants.USE_ROBOT_RELATIVE) {
        drivebase.setDefaultCommand(
            drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
      } else {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        // m_shooter.setDefaultCommand(m_shooter.SpeedUpShooterCommand());
      }
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      dc().start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      dc().button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      dc().button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    // driverXbox.leftBumper().onTrue(Commands.none());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // } else
    // {
    // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.start().whileTrue(Commands.none());
    // driverXbox.back().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // }
  }

  private double aimTolerance(double distance) {
    if (distance < 2)
      return 5.0;
    else if (distance < 3.5)
      return 2.0;
    return 1.0;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    Command selected = loggedAutoChooser.get();
    if (selected == null)
      return Commands.none();

    // The flip choice is applied here rather than by rebuilding the chooser, so the
    // chooser and its logged NT key are only ever created once.
    if (selected instanceof PathPlannerAuto && Boolean.TRUE.equals(flipChooser.getSelected())) {
      return new PathPlannerAuto(selected.getName(), true);
    }

    // String selectedName = loggedAutoChooser.get().getName();

    // put the main path (swipe) and the recovery path
    // if (selectedName.equals("Swipe Correction Test")) {
    // return Commands.sequence(
    // followWithRecovery("LT Swipe", "Through LT"),
    // makeAutoShootCommand());
    // }

    return selected;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setUseMegaTag2(boolean use) {
    drivebase.useMegaTag2 = use;
  }

  public void logControllerInputs() {
    // Driver left stick X (-1..1).
    Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
    // Driver left stick Y (-1..1).
    Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
    // Driver right stick X (-1..1).
    Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
    // Driver right stick Y (-1..1).
    Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
    // Driver left trigger (0..1).
    Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
    // Driver right trigger (0..1).
    Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());

    // Operator left stick X (-1..1).
    Logger.recordOutput("Input/Operator/LeftX", operatorXbox.getLeftX());
    // Operator left stick Y (-1..1).
    Logger.recordOutput("Input/Operator/LeftY", operatorXbox.getLeftY());
    // Operator right stick X (-1..1).
    Logger.recordOutput("Input/Operator/RightX", operatorXbox.getRightX());
    // Operator right stick Y (-1..1).
    Logger.recordOutput("Input/Operator/RightY", operatorXbox.getRightY());
    // Operator left trigger (0..1).
    Logger.recordOutput("Input/Operator/LeftTrigger", operatorXbox.getLeftTriggerAxis());
    // Operator right trigger (0..1).
    Logger.recordOutput("Input/Operator/RightTrigger", operatorXbox.getRightTriggerAxis());

    // --- Shooting sequence state ---
    boolean rtHeld = dc().rightTrigger().getAsBoolean();
    Logger.recordOutput("Shooting/RTHeld", rtHeld);
    Logger.recordOutput("Shooting/InAllianceZone", isInAllianceZone());

    if (aimAtTarget != null) {
      Logger.recordOutput("Shooting/AimLock1Deg",
          aimAtTarget.swerveInputStream.aimLock(Degrees.of(1.0)).getAsBoolean());
      Logger.recordOutput("Shooting/AimLock3Deg",
          aimAtTarget.swerveInputStream.aimLock(Degrees.of(3.0)).getAsBoolean());
    }
    Logger.recordOutput("Shooting/InAllianceZone", drivebase.isInAllianceZone());
    Logger.recordOutput("Shooting/InNeutralZone", drivebase.isInNeutralZone());
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Meters.of(FieldConstants.LinesVertical.allianceZone);
    Distance redZone = Meters.of(FieldConstants.LinesVertical.oppAllianceZone);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isInOpponentZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Meters.of(FieldConstants.LinesVertical.allianceZone);
    Distance redZone = Meters.of(FieldConstants.LinesVertical.oppAllianceZone);

    if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isOnAllianceOutpostSide() {
    Alliance alliance = getAlliance();
    Distance midLine = Inches.of(158.84375);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) {
      return true;
    }

    return false;
  }

  // private void configureFuelSim() {
  // fuelSim = new FuelSim();
  // fuelSim.spawnStartingFuel();

  // fuelSim.start();
  // SmartDashboard.putData(Commands.runOnce(() -> {
  // fuelSim.clearFuel();
  // fuelSim.spawnStartingFuel();
  // })
  // .withName("Reset Fuel")
  // .ignoringDisable(true));
  // }

  // private void configureFuelSimRobot() {
  // fuelSim.registerRobot(
  // Dimensions.FULL_WIDTH.in(Meters),
  // Dimensions.FULL_LENGTH.in(Meters),
  // Dimensions.BUMPER_HEIGHT.in(Meters),
  // drivebase::getPose,
  // drivebase::getFieldVelocity);

  // // fuelSim.registerIntake(
  // // -Dimensions.FULL_LENGTH.div(2).in(Meters),
  // // Dimensions.FULL_LENGTH.div(2).in(Meters),
  // // -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
  // // -Dimensions.FULL_WIDTH.div(2).in(Meters),
  // // () -> m_slapdown.isRightDeployed() && ableToIntake.getAsBoolean(),
  // // intakeCallback);
  // }

  private double computeDynamicLookaheadSeconds() {
    // Read robot field velocities (from SwerveSubsystem)
    var chassisSpeeds = drivebase.getFieldVelocity(); // ChassisSpeeds

    double omega = Math.abs(chassisSpeeds.omegaRadiansPerSecond);
    double speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    // Simple linear combination of yaw rate and translation speed
    double lookahead = Constants.LOOKAHEAD_BASE_SEC + Constants.LOOKAHEAD_K_OMEGA * omega
        + Constants.LOOKAHEAD_K_V * speed;

    // Clamp to safe range
    lookahead = Math.min(Math.max(lookahead, Constants.LOOKAHEAD_MIN_SEC), Constants.LOOKAHEAD_MAX_SEC);

    return lookahead;
  }

  /**
   * Checks if the robot heading is within a tolerance of the angle toward a
   * target pose.
   */
  private boolean isAimedAt(Pose2d target, double toleranceDegrees) {
    Pose2d robot = drivebase.getPose();
    double targetAngle = Math.toDegrees(Math.atan2(
        target.getY() - robot.getY(),
        target.getX() - robot.getX()));
    double currentAngle = robot.getRotation().getDegrees();
    double error = Math.abs(currentAngle - targetAngle);
    error = error % 360;
    if (error > 180)
      error = 360 - error;
    return error <= toleranceDegrees;
  }

}