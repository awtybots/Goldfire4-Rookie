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
import frc.robot.commands.AimAtHub;
import frc.robot.commands.AimAtFerry;
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
// import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
  // private final Climber m_climber = new Climber();
  private final Kicker m_kicker = new Kicker();
  private final Slapdown m_slapdown = new Slapdown();

  // Helper Subsystems
  private final HubTrackerSubsystem m_hubtracker = new HubTrackerSubsystem(drivebase, driverXbox);

  // private final ObjectDetection m_ObjectDetection = new ObjectDetection();

  // Factory for ControlAllShooting instances. Create a fresh instance for each
  // composition to avoid WPILib's "composed commands may not be reused" error.
  // private ControlAllShooting makeVariableShoot() {
  //   return new ControlAllShooting(drivebase::getCachedDynamicHubLocation, m_shooter, drivebase::getPose);
  // }

  // public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of
  // FuelSim

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> loggedAutoChooser;
  // Add this field at the top of RobotContainer (alongside your other fields)
  private SendableChooser<Boolean> flipChooser = new SendableChooser<>();

  // Driver chooser: "David" = port 0 drives, "Asier" = port 1 drives
  private final SendableChooser<String> driverChooser = new SendableChooser<>();

  // -----------------------------------------------------------------------
  // SwerveInputStreams — built in configureBindings() so they reference
  // whichever controller was selected as driver via dc().
  // -----------------------------------------------------------------------

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

  private AimAtHub aimAtHub;
  private AimAtFerry aimAtFerry;
  private PathConstraints autoConstraints;

  SwerveInputStream aimAtHubStream;
  SwerveInputStream aimAtFerryStream;
  // ========= DRIVER TRIGGERS ===========
  // Parallel Commands
  private Trigger RTtransfer_kick_shoot; // index to kicker, kick, agitate, and shoot only when up to speed
  private Trigger RBFerry; // Run hopper and kicker in reverse
  private Trigger LBretract_and_stop; // retract 4 bar and stop intake
  private Trigger PRDrivetoRightTrench; // Drive to right trench
  private Trigger PLDriveToPose; // run hopper in reverse and kick backwards to unjam

  // Shooter
  private Trigger LT_Intake;

  // Intake
  private Trigger X_runIntake;
  private Trigger A_runOuttake;

  // Pushout
  private Trigger Y_extendIntake;
  private Trigger B_agitate;

  // Climber
  private Trigger Climb;
  private Trigger ClimbDown;

  // ========= OPERATOR TRIGGERS ===========
  // Shooter
  private Trigger LT_OP_1900Shot; // just shoot
  private Trigger RT_OP_VariableShoot; // Shoot, Kick, Index, Agitate, and Run Intake

  // Get to Shooter
  private Trigger RB_OP_Pass; // kick, index
  private Trigger LB_OP_unjam; // unjam

  // Intake
  private Trigger X_OP_intake; // intake fuel
  private Trigger A_OP_outtake; // outtake fuel

  // Pushout
  private Trigger Y_OP_extendIntake; // push out
  private Trigger B_OP_reteactIntake; // pull in
  private Trigger POVLEFT_OP_agitate; // agitate 

  // Climber / Vision
  private Trigger POVUP_OP_FrontLimelight;
  private Trigger POVLEFT_OP_LeftLimelight;
  private Trigger POVRIGHT_OP_VisionToggle;
  private Trigger POVDown_OP_BackLimelight; // toggle vision

  // -----------------------------------------------------------------------
  // Helpers: resolve which physical controller acts as "driver" vs "operator"
  // based on the SmartDashboard chooser selection.  
  // -----------------------------------------------------------------------
  private boolean isAsierSelected() {
    String selected = driverChooser.getSelected();
    return selected != null && selected.equals("Asier");
  }

  /** Returns the controller that should be treated as the driving controller. */
  private CommandXboxController dc() {
    return isAsierSelected() ? operatorXbox : driverXbox;
  }

  /** Returns the controller that should be treated as the operator controller. */
  private CommandXboxController oc() {
    return isAsierSelected() ? driverXbox : operatorXbox;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // ---- Driver chooser: put on SmartDashboard before configureBindings() ----
    driverChooser.setDefaultOption("David", "David");
    driverChooser.addOption("Asier", "Asier");
    SmartDashboard.putData("Driver:", driverChooser);

    // Configure the trigger bindings
    configureBindings();

    // configureFuelSim();
    // configureFuelSimRobot();
    // Triggers for auto aim/pass poses

    DriverStation.silenceJoystickConnectionWarning(true);
    SmartDashboard.putNumber("Heading Bias Deg", 0.0);
    // SmartDashboard.putBoolean("Is Shooter Running", m_shooter.isShooterRunning());
    // Tunable gain: radians of bias -> radians/sec of angular velocity
    SmartDashboard.putNumber("Heading Bias Gain", 0);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));


    // setup the flip chooser
    flipChooser.setDefaultOption("Not Flipped", false);
    flipChooser.addOption("Flipped", true);
    SmartDashboard.putData("Flip Auto", flipChooser);

    flipChooser.onChange((Boolean flip) -> {
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
          autoStream -> autoStream.map(auto -> {
            auto = new PathPlannerAuto(auto.getName(), flip);
            return auto;
          }));
      autoChooser.setDefaultOption("Do Nothing", Commands.none());
      SmartDashboard.putData("Auto Chooser", autoChooser);
      loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
    });

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        autoStream -> autoStream.map(auto -> {
          auto = new PathPlannerAuto(auto.getName(), flipChooser.getSelected());
          return auto;
        }));
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

    // Build all SwerveInputStreams here using dc() so they reference the
    // correct driver controller based on the chooser selection.
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> dc().getLeftY() * -1,
        () -> dc().getLeftX() * -1)
        .withControllerRotationAxis(() -> dc().getRightX() * -1)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(1.0)
        .allianceRelativeControl(true);

    oc().rightTrigger().whileTrue(
    Commands.parallel(
        m_shooter.setShooterSpeedCommand(1200),
        Commands.sequence(
            Commands.waitUntil(() -> m_shooter.isShooterFast()),
            Commands.parallel(
                m_kicker.kickCommand(),
                m_hopper.runBeltsToConveyorCommand()
            )
        )
    )
);
    
/* 
    dc().rightTrigger().whileTrue(Commands.defer(() -> {
      if (isInAllianceZone()) {
        aimAtHub = new AimAtHub(drivebase, driveAngularVelocity,
            dc()::getLeftX, dc()::getLeftY, dc()::getRightX);
        return aimAtHub;
      } else {
        aimAtFerry = new AimAtFerry(drivebase, driveAngularVelocity);
        return aimAtFerry;
      }
    }, Set.of(drivebase)));
 */
    driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(dc()::getRightX, dc()::getRightY)
        .headingWhile(true);

    driveRobotOriented = driveAngularVelocity.copy()
        .robotRelative(true)
        .allianceRelativeControl(false);

    driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -dc().getLeftY(),
        () -> -dc().getLeftX())
        .withControllerRotationAxis(() -> dc().getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // Derive the heading axis with math!
    driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(
            () -> Math.sin(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos(dc().getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true)
        .translationHeadingOffset(true)
        .translationHeadingOffset(Rotation2d.fromDegrees(0));

    aimAtHubStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0)
        .aim(() -> drivebase.getCachedDynamicHubLocation())
        .aimWhile(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds))
        .aimFeedforward(0.0001, 0.0001, 0.00013)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true);

    aimAtFerryStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0)
        .aim(() -> drivebase.getCachedDynamicFerryLocation())
        .aimWhile(true)
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds))
        .aimFeedforward(0.0001, 0.0001, 0.00013)
        .aimHeadingOffset(Rotation2d.fromDegrees(180))
        .aimHeadingOffset(true);

    // ========= DRIVER TRIGGERS ===========
    // Parallel Commands
    RTtransfer_kick_shoot = dc().rightTrigger(); // index to kicker, kick, agitate, and shoot only when up to speed
    RBFerry = dc().rightBumper(); // Run hopper and kicker in reverse
    LBretract_and_stop = dc().leftBumper(); // retract 4 bar and stop intake
    PRDrivetoRightTrench = dc().povRight(); // Drive to right trench
    PLDriveToPose = dc().povLeft(); // run hopper in reverse and kick backwards to unjam


    dc().a().whileTrue(m_hopper.runBeltsToConveyorCommand());
    
    // Shooter
    LT_Intake = dc().leftTrigger();

    // Intake
    X_runIntake = dc().x();
    A_runOuttake = dc().a();

    // Pushout
    Y_extendIntake = dc().y();
    B_agitate = dc().b();

    // Climber
    Climb = dc().povUp();
    ClimbDown = dc().povDown();

    // ========= OPERATOR TRIGGERS ===========
    // Shooter
    LT_OP_1900Shot = oc().leftTrigger(); // just shoot
    RT_OP_VariableShoot = oc().rightTrigger(); // Shoot, Kick, Index, Agitate, and Run Intake

    Trigger ResetEncoder = oc().start();

    // Get to Shooter
    RB_OP_Pass = oc().rightBumper(); // kick, index
    LB_OP_unjam = oc().leftBumper(); // unjam

    // Intake
    X_OP_intake = oc().x(); // intake fuel
    A_OP_outtake = oc().a(); // outtake fuel

    // Pushout
    Y_OP_extendIntake = oc().y(); // push out
    B_OP_reteactIntake = oc().b(); // pull in
    POVLEFT_OP_agitate = oc().povLeft(); // agitate

    // Climber / Vision
    POVUP_OP_FrontLimelight = oc().povUp();
    POVLEFT_OP_LeftLimelight = oc().povLeft();
    POVRIGHT_OP_VisionToggle = oc().povRight();
    POVDown_OP_BackLimelight = oc().povDown(); // toggle vision

    Command driveFieldOrientedDirectAngle = drivebase
        .driveFieldOriented(() -> applyHeadingBias(driveDirectAngle.get()));
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
        () -> applyHeadingBias(driveAngularVelocity.get()));
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(
        () -> applyHeadingBias(driveDirectAngleKeyboard.get()));
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(
        () -> applyHeadingBias(driveAngularVelocityKeyboard.get()));
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
    // oc().a().whileTrue(m_shooter.sysIdQuasistaticForward());
    // // SysId: run shooter quasistatic reverse.
    // oc().b().whileTrue(m_shooter.sysIdQuasistaticReverse());
    // // SysId: run shooter dynamic forward.
    // oc().x().whileTrue(m_shooter.sysIdDynamicForward());
    // // SysId: run shooter dynamic reverse.
    // oc().y().whileTrue(m_shooter.sysIdDynamicReverse());

    // new Trigger(() -> isInAllianceZone()
    //     && DriverStation.isTeleop())
    //     .onTrue(Commands.runOnce(() -> m_shooter.setDefaultCommand(m_shooter.setAllianceIdle())));
    // new Trigger(() -> !isInAllianceZone()
    //     && DriverStation.isTeleopEnabled())
    //     .onTrue(Commands.runOnce(() -> m_shooter.setDefaultCommand(m_shooter.setNeutralIdle())));
   //  m_shooter.setDefaultCommand(m_shooter.setAllianceIdle().onlyWhile(() -> DriverStation.isTeleopEnabled()));

    // m_intake.setDefaultCommand(m_intake.runDefaultCommand());
    // m_kicker.setDefaultCommand(m_kicker.runDefaultCommand());
    // // m_slapdown.setDefaultCommand(m_slapdown.runDefaultCommand());
    // m_hopper.setDefaultCommand(m_hopper.runDefaultCommand());

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
    if (DriverStation.isTest()) {
      if (Constants.USE_ROBOT_RELATIVE) {
        drivebase.setDefaultCommand(
            drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
      } else {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides
        // drive command above!
      }
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

    if (aimAtHub != null) {
      Logger.recordOutput("Shooting/AimLock1Deg",
          aimAtHub.swerveInputStream.aimLock(Degrees.of(1.0)).getAsBoolean());
      Logger.recordOutput("Shooting/AimLock3Deg",
          aimAtHub.swerveInputStream.aimLock(Degrees.of(3.0)).getAsBoolean());
    }
    if (aimAtFerry != null) {
      Logger.recordOutput("Shooting/FerryAimLock3Deg",
          aimAtFerry.swerveInputStream.aimLock(Degrees.of(3.0)).getAsBoolean());
    }
  }

  private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
    // Toggle to enable heading bias; false means pass-through.
    boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
    if (!headingBiasEnabled) {
      return speeds;
    }
    // Requested heading bias in degrees; 0 means disabled.
    double biasDeg = SmartDashboard.getNumber("Heading Bias Deg", 0.0);
    // Gain mapping bias radians -> added omega (rad/sec).
    double gain = SmartDashboard.getNumber("Heading Bias Gain", 0.0);

    // Default to normal driving (no bias).
    double omega = speeds.omegaRadiansPerSecond;
    if (biasDeg != 0.0 && gain != 0.0) {
      // Convert degrees to radians, then scale into an omega offset.
      double biasRad = Units.degreesToRadians(biasDeg);
      double additionalOmega = gain * biasRad;
      // Leave vx/vy alone; only add a small angular velocity component.
      omega += additionalOmega;
    }

    return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isInOpponentZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

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